from setuptools import find_packages, setup

package_name = 'ai_worker_manipulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data',
            ['ai_worker_manipulation/data/object_lut.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamin',
    maintainer_email='chlgkals0730@gmail.com',
    description='Manipulation stack for the 2026 Humanoid Challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_moveit_client      = ai_worker_manipulation.tests.test_moveit_client:main',
            'test_arm_motion         = ai_worker_manipulation.tests.test_arm_motion:main',
            'test_gripper            = ai_worker_manipulation.tests.test_gripper:main',
            'test_pick_and_place     = ai_worker_manipulation.tests.test_pick_and_place:main',
            'test_tf_gpd             = ai_worker_manipulation.tests.test_tf_gpd:main',
            'gpd_wrist               = ai_worker_manipulation.gpd_wrist_node:main',
            'move_wrist_capture_pose = ai_worker_manipulation.tests.move_wrist_capture_pose:main',
            'demo_gpd_grasp          = ai_worker_manipulation.tests.demo_gpd_grasp:main',
            'test_gpd_wrist150       = ai_worker_manipulation.tests.test_gpd_wrist150:main',
            'demo_gpd_pick_place     = ai_worker_manipulation.tests.demo_gpd_pick_place:main',
        ],
    },
)
