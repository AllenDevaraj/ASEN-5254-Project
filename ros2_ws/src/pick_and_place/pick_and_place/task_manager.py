#!/usr/bin/env python3

"""
Task and Motion Planning (TAMP) TaskManager for Dual-Arm Robot System.

This module implements a symbolic task planner that maintains world state using predicates
and generates action sequences to achieve goals (e.g., making objects graspable).

Author: ASEN-5254 Project
Date: 2025
"""

from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class PredicateType(Enum):
    """Types of predicates in the world state."""
    ON = "on"  # on(obj_top, obj_bottom) - obj_top is on top of obj_bottom
    CLEAR = "clear"  # clear(obj) - nothing is on top of obj
    HOLDING = "holding"  # holding(arm, obj) - arm is holding obj
    HAND_EMPTY = "hand_empty"  # hand_empty(arm) - arm is not holding anything
    ON_TABLE = "on_table"  # on_table(obj) - obj is on the table


@dataclass
class Predicate:
    """Represents a single predicate (fact) in the world state."""
    predicate_type: PredicateType
    args: List[str]  # Arguments for the predicate (e.g., ['obj1', 'obj2'] for on(obj1, obj2))
    
    def __hash__(self):
        return hash((self.predicate_type, tuple(self.args)))
    
    def __eq__(self, other):
        if not isinstance(other, Predicate):
            return False
        return self.predicate_type == other.predicate_type and self.args == other.args
    
    def __str__(self):
        args_str = ", ".join(self.args)
        return f"{self.predicate_type.value}({args_str})"


@dataclass
class Action:
    """Represents an action/operator with preconditions and effects."""
    name: str  # e.g., "pick_up", "unstack", "put_down"
    params: List[str]  # Parameters: [arm, obj] or [arm, obj_top, obj_bottom]
    preconditions: List[Predicate]  # Must be true before action can execute
    effects_add: List[Predicate]  # Predicates to add (positive effects)
    effects_remove: List[Predicate]  # Predicates to remove (negative effects)
    
    def __str__(self):
        params_str = ", ".join(self.params)
        return f"{self.name}({params_str})"


class TaskManager:
    """
    Task and Motion Planning Manager.
    
    Maintains world state using predicates and generates action plans to achieve goals.
    """
    
    def __init__(self, objects: Dict, table_height: float = 0.2, clearance_threshold: float = 0.01):
        """
        Initialize TaskManager.
        
        Args:
            objects: Dictionary of objects with 'position', 'size', etc.
            table_height: Height of the table surface (default: 0.2m)
            clearance_threshold: Z-distance threshold to determine if object is "on" another (default: 0.01m)
        """
        self.objects = objects
        self.table_height = table_height
        self.clearance_threshold = clearance_threshold
        
        # World state: Set of predicates that are currently true
        self.state: Set[Predicate] = set()
        
        # Initialize state from object poses
        self._initialize_state()
        
    def _initialize_state(self):
        """Initialize world state by inferring predicates from object positions."""
        self.state.clear()
        
        # Get all object names
        obj_names = list(self.objects.keys())
        
        # Initialize: all objects are on table, all are clear, all arms are empty
        for obj_name in obj_names:
            self.state.add(Predicate(PredicateType.ON_TABLE, [obj_name]))
            self.state.add(Predicate(PredicateType.CLEAR, [obj_name]))
        
        # Assume both arms start empty
        self.state.add(Predicate(PredicateType.HAND_EMPTY, ["panda1"]))
        self.state.add(Predicate(PredicateType.HAND_EMPTY, ["panda2"]))
        
        # Infer "on" relationships from object positions
        self._infer_on_relationships()
        
    def _infer_on_relationships(self):
        """
        Infer which objects are on top of others by comparing Z positions and XY proximity.
        Updates state accordingly (removes on_table and clear predicates as needed).
        """
        obj_names = list(self.objects.keys())
        
        for obj_top_name in obj_names:
            obj_top = self.objects[obj_top_name]
            top_pos = obj_top.get('position', [0, 0, 0])
            top_size = obj_top.get('size', [0.05, 0.05, 0.05])
            top_center_z = top_pos[2]
            top_bottom_z = top_center_z - (top_size[2] / 2.0)
            
            # Check if this object is on top of another object (not the table)
            for obj_bottom_name in obj_names:
                if obj_top_name == obj_bottom_name:
                    continue
                
                obj_bottom = self.objects[obj_bottom_name]
                bottom_pos = obj_bottom.get('position', [0, 0, 0])
                bottom_size = obj_bottom.get('size', [0.05, 0.05, 0.05])
                bottom_center_z = bottom_pos[2]
                bottom_top_z = bottom_center_z + (bottom_size[2] / 2.0)
                
                # Check XY proximity (within half the width of the bottom object)
                top_x, top_y = top_pos[0], top_pos[1]
                bottom_x, bottom_y = bottom_pos[0], bottom_pos[1]
                
                # Use the larger dimension for proximity check
                bottom_max_size = max(bottom_size[0], bottom_size[1])
                xy_distance = ((top_x - bottom_x)**2 + (top_y - bottom_y)**2)**0.5
                
                # Check if top object is on bottom object
                # CRITICAL: Top object must be ABOVE bottom object (top_bottom_z > bottom_top_z)
                # AND they must be close vertically (within threshold)
                z_gap = top_bottom_z - bottom_top_z  # Positive if top is above bottom
                
                # Only create relationship if:
                # 1. Top object is above bottom object (z_gap >= 0, allowing small tolerance)
                # 2. They're close enough vertically (z_gap < threshold)
                # 3. They're close enough horizontally (xy_distance < half width)
                if (z_gap >= -0.01 and z_gap < self.clearance_threshold and 
                    xy_distance < (bottom_max_size / 2.0)):
                    # obj_top is on obj_bottom
                    # Remove: on_table(obj_top), clear(obj_bottom)
                    # Add: on(obj_top, obj_bottom)
                    self.state.discard(Predicate(PredicateType.ON_TABLE, [obj_top_name]))
                    self.state.discard(Predicate(PredicateType.CLEAR, [obj_bottom_name]))
                    self.state.add(Predicate(PredicateType.ON, [obj_top_name, obj_bottom_name]))
                    break  # Each object can only be on one thing
    
    def update_state_from_poses(self):
        """Re-infer world state from current object poses (call after objects move)."""
        self._initialize_state()
    
    def is_true(self, predicate: Predicate) -> bool:
        """Check if a predicate is true in the current state."""
        return predicate in self.state
    
    def add_predicate(self, predicate: Predicate):
        """Add a predicate to the state."""
        self.state.add(predicate)
    
    def remove_predicate(self, predicate: Predicate):
        """Remove a predicate from the state."""
        self.state.discard(predicate)
    
    def apply_action(self, action: Action) -> bool:
        """
        Apply an action to the world state (if preconditions are met).
        
        Returns:
            True if action was applied, False if preconditions not met.
        """
        # Check preconditions
        for prec in action.preconditions:
            if not self.is_true(prec):
                return False
        
        # Apply effects (remove first, then add)
        for effect_remove in action.effects_remove:
            self.remove_predicate(effect_remove)
        
        for effect_add in action.effects_add:
            self.add_predicate(effect_add)
        
        return True
    
    def get_plan_to_grasp(self, target_obj: str, arm: str = "panda1") -> List[Action]:
        """
        Generate a plan to make target_obj graspable and pick it up.
        
        Logic:
        1. Check if target_obj is clear.
        2. If yes, return [pick_up(arm, target_obj)].
        3. If no, find what is on the target.
        4. Recursively clear obstructions.
        5. Generate plan: [unstack(...), put_down(...), pick_up(...)].
        
        Args:
            target_obj: Name of object to grasp (e.g., "green_hollow")
            arm: Arm to use for manipulation (default: "panda1")
        
        Returns:
            List of Actions to execute in order.
        """
        plan = []
        
        # Check if target is already clear
        clear_pred = Predicate(PredicateType.CLEAR, [target_obj])
        if self.is_true(clear_pred):
            # Target is clear, can pick up directly
            pick_action = self._create_pick_up_action(arm, target_obj)
            if pick_action:
                plan.append(pick_action)
            return plan
        
        # Target is not clear - find what's on it
        obstruction = self._find_obstruction(target_obj)
        if obstruction is None:
            # Shouldn't happen if clear is false, but handle it
            return []
        
        # Recursively clear the obstruction
        unstack_plan = self._get_plan_to_clear(obstruction, arm)
        plan.extend(unstack_plan)
        
        # Now unstack the obstruction from target
        unstack_action = self._create_unstack_action(arm, obstruction, target_obj)
        if unstack_action:
            plan.append(unstack_action)
        
        # Put obstruction down on table
        put_down_action = self._create_put_down_action(arm, obstruction)
        if put_down_action:
            plan.append(put_down_action)
        
        # Finally, pick up the target
        pick_action = self._create_pick_up_action(arm, target_obj)
        if pick_action:
            plan.append(pick_action)
        
        return plan
    
    def _find_obstruction(self, obj: str) -> Optional[str]:
        """Find what object is on top of obj (returns None if nothing is on it)."""
        for pred in self.state:
            if pred.predicate_type == PredicateType.ON and pred.args[1] == obj:
                return pred.args[0]  # Return the object that is on top
        return None
    
    def _get_plan_to_clear(self, obj: str, arm: str, visited: Optional[Set[str]] = None) -> List[Action]:
        """
        Recursively generate plan to clear obj (remove anything on top of it).
        Returns empty list if obj is already clear.
        
        Args:
            obj: Object to clear
            arm: Arm to use
            visited: Set of objects already visited (to prevent infinite recursion from circular relationships)
        """
        if visited is None:
            visited = set()
        
        plan = []
        
        # CRITICAL: Detect cycles to prevent infinite recursion
        if obj in visited:
            # Circular relationship detected - log warning and return empty plan
            # This should not happen if _infer_on_relationships() is correct, but guard against it
            return plan
        
        visited.add(obj)
        
        # Check if obj is already clear
        clear_pred = Predicate(PredicateType.CLEAR, [obj])
        if self.is_true(clear_pred):
            return plan
        
        # Find obstruction
        obstruction = self._find_obstruction(obj)
        if obstruction is None:
            return plan
        
        # Recursively clear the obstruction (pass visited set to track cycle)
        sub_plan = self._get_plan_to_clear(obstruction, arm, visited.copy())
        plan.extend(sub_plan)
        
        # Unstack obstruction from obj
        unstack_action = self._create_unstack_action(arm, obstruction, obj)
        if unstack_action:
            plan.append(unstack_action)
        
        # Put obstruction down
        put_down_action = self._create_put_down_action(arm, obstruction)
        if put_down_action:
            plan.append(put_down_action)
        
        return plan
    
    def _create_pick_up_action(self, arm: str, obj: str) -> Optional[Action]:
        """Create a pick_up action with proper preconditions and effects."""
        # Preconditions
        preconditions = [
            Predicate(PredicateType.CLEAR, [obj]),
            Predicate(PredicateType.ON_TABLE, [obj]),
            Predicate(PredicateType.HAND_EMPTY, [arm])
        ]
        
        # Effects (add)
        effects_add = [
            Predicate(PredicateType.HOLDING, [arm, obj])
        ]
        
        # Effects (remove)
        effects_remove = [
            Predicate(PredicateType.ON_TABLE, [obj]),
            Predicate(PredicateType.CLEAR, [obj]),
            Predicate(PredicateType.HAND_EMPTY, [arm])
        ]
        
        return Action("pick_up", [arm, obj], preconditions, effects_add, effects_remove)
    
    def _create_unstack_action(self, arm: str, obj_top: str, obj_bottom: str) -> Optional[Action]:
        """Create an unstack action with proper preconditions and effects."""
        # Preconditions
        preconditions = [
            Predicate(PredicateType.ON, [obj_top, obj_bottom]),
            Predicate(PredicateType.CLEAR, [obj_top]),
            Predicate(PredicateType.HAND_EMPTY, [arm])
        ]
        
        # Effects (add)
        effects_add = [
            Predicate(PredicateType.HOLDING, [arm, obj_top]),
            Predicate(PredicateType.CLEAR, [obj_bottom])
        ]
        
        # Effects (remove)
        effects_remove = [
            Predicate(PredicateType.ON, [obj_top, obj_bottom]),
            Predicate(PredicateType.HAND_EMPTY, [arm])
        ]
        
        return Action("unstack", [arm, obj_top, obj_bottom], preconditions, effects_add, effects_remove)
    
    def _create_put_down_action(self, arm: str, obj: str, location: str = "table") -> Optional[Action]:
        """Create a put_down action with proper preconditions and effects."""
        # Preconditions
        preconditions = [
            Predicate(PredicateType.HOLDING, [arm, obj])
        ]
        
        # If placing on another object, it must be clear
        if location != "table":
            preconditions.append(Predicate(PredicateType.CLEAR, [location]))
        
        # Effects (add)
        effects_add = [
            Predicate(PredicateType.ON_TABLE, [obj]),
            Predicate(PredicateType.CLEAR, [obj]),
            Predicate(PredicateType.HAND_EMPTY, [arm])
        ]
        
        # Effects (remove)
        effects_remove = [
            Predicate(PredicateType.HOLDING, [arm, obj])
        ]
        
        return Action("put_down", [arm, obj, location], preconditions, effects_add, effects_remove)
    
    def get_state_string(self, include_positions: bool = False) -> str:
        """Get a human-readable string of the current world state."""
        lines = []
        for pred in sorted(self.state, key=lambda p: (p.predicate_type.value, p.args)):
            lines.append(str(pred))
        # Only add object positions if explicitly requested (expensive for large object sets)
        if include_positions:
            lines.append("\n=== Object Positions ===")
            for obj_name, obj_data in self.objects.items():
                pos = obj_data.get('position', [0, 0, 0])
                size = obj_data.get('size', [0.05, 0.05, 0.05])
                lines.append(f"{obj_name}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), size={size}")
        return "\n".join(lines)
    
    def print_state(self):
        """Print the current world state."""
        print("=== Current World State ===")
        print(self.get_state_string())
        print("===========================")

