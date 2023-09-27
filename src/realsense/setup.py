from setuptools import find_packages, setup

package_name = 'realsense'

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
    maintainer='robotics',
    maintainer_email='lmerc024@fiu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "intel_pub=realsense.intel_pub:main",
            "intel_sub=realsense.intel_sub:main",
            "intel_pub2=realsense.intel_pub2:main",
            "intel_sub2=realsense.intel_sub2:main",
            "pointcloud_visualizer_sub_node=realsense.pointcloud_visualizer_sub_node:main",
            "intel_sub3=realsense.intel_sub3:main"
        ],
    },
)
