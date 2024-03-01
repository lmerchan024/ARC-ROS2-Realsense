from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Merchan',
    maintainer_email='lmerc024@fiu.edu',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'middleman=py_pubsub.intermediary_member_function:main',
            
            'pntCldMidMan=py_pubsub.pntCld_middleman_node:main',
            'pntCldConverter=py_pubsub.pntCld_converter_node:main',
            'pntCldCreator=py_pubsub.pntCld_creator_example_node:main',

            'numpyO3dConverter=py_pubsub.numpy_to_o3d_pointcloud_template_script:main',
            'numpyRos2Converter=py_pubsub.numpy_to_ros2_pointcloud_template_script:main',
            'ros2NumpyConverter=py_pubsub.ros2_pointcloud_to_numpy_template_script:main',
            'o3dNumpyConverter=py_pubsub.o3d_pointcloud_to_numpy_template_script:main',

            'ros2_cylinder_detector=py_pubsub.ros2_cylinder_detector:main',
            'ros2_pointcloud_to_numpy_template_script_ver2=py_pubsub.ros2_pointcloud_to_numpy_template_script_ver2:main',
            'ros2_pointcloud_to_numpy_template_script_ver3=py_pubsub.ros2_pointcloud_to_numpy_template_script_ver3:main',
            'ros2_pointcloud_to_numpy_template_script_ver4=py_pubsub.ros2_pointcloud_to_numpy_template_script_ver4:main',
            'ros2_pointcloud_to_numpy_template_script_ver5=py_pubsub.ros2_pointcloud_to_numpy_template_script_ver5:main',
            'ros2_pointcloud_to_numpy_template_script_ver4b=py_pubsub.ros2_pointcloud_to_numpy_template_script_ver4b:main',

            'plane_getter=py_pubsub.plane_getter:main',
        ],
    },
)
