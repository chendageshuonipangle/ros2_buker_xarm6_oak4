from setuptools import find_packages, setup

package_name = 'oak_yolo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install model archive so it is available from the package share directory
        ('share/' + package_name + '/models', ['oak_yolo_py/yolo11m.rvc4.tar.xz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc-24',
    maintainer_email='pc-24@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'oak_yolo_node = oak_yolo_py.oak_yolo_node:main',
            'oak_chessboard_node = oak_yolo_py.oak_chessboard_node:main',
        ],
    },
)
