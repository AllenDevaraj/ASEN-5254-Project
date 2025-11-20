from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pick_and_place'

# Collect all model files
model_files = []
for model_dir in glob('models/*'):
    if os.path.isdir(model_dir):
        model_name = os.path.basename(model_dir)
        for root, dirs, files in os.walk(model_dir):
            if files:
                rel_path = os.path.relpath(root, 'models')
                install_path = os.path.join('share', package_name, 'models', rel_path)
                file_list = [os.path.join(root, f) for f in files]
                model_files.append((install_path, file_list))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elena Oikonomou',
    maintainer_email='root@todo.todo',
    description='Pick and place application for Panda robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = pick_and_place.controller:main',
            'panda_ik_gui = pick_and_place.gui_end_effector:main',
            'panda1_ik_gui = pick_and_place.gui_end_effector:panda1_main',
            'panda2_ik_gui = pick_and_place.gui_end_effector:panda2_main',
            'dual_arm_gui = pick_and_place.dual_arm_gui:main',
        ],
    },

)

