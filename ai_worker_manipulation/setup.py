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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamin',
    maintainer_email='chlgkals0730@gmail.com',
    description='Manipulation stack for the 2026 Humanoid Challenge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_move_to_pose = ai_worker_manipulation.tests.test_move_to_pose:main',
            'move_home = ai_worker_manipulation.tests.move_home:main',
            'move_to_pose = ai_worker_manipulation.tests.move_to_pose:main',
            'demo_0513 = ai_worker_manipulation.tests.demo_0513:main',
            'demo_0520 = ai_worker_manipulation.tests.demo_0520:main',
            'demo_0521 = ai_worker_manipulation.tests.demo_0521:main',
            'pc_transformer = ai_worker_manipulation.skill_primitives.point_cloud_transformer_node:main',
            'gpd_wrist = ai_worker_manipulation.gpd_wrist_node:main',
            'demo_gpd_grasp = ai_worker_manipulation.tests.demo_gpd_grasp:main',
            'move_wrist_capture_pose = ai_worker_manipulation.tests.move_wrist_capture_pose:main',
            'gpd_grasp_publisher = ai_worker_manipulation.gpd_grasp_publisher:main',
            'test_arm_motion = ai_worker_manipulation.tests.test_arm_motion:main',
            'test_gripper = ai_worker_manipulation.tests.test_gripper:main',
            'test_moveit_client = ai_worker_manipulation.tests.test_moveit_client:main',
            'test_pick_and_place = ai_worker_manipulation.tests.test_pick_and_place:main',
            'test_tf_gpd = ai_worker_manipulation.tests.test_tf_gpd:main',
            'test_lift = ai_worker_manipulation.tests.test_lift:main',
        ],
    },
)
