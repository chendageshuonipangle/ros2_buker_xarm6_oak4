from setuptools import find_packages, setup


package_name = "xarm_oak_handeye"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/handeye_target_transform.launch.py"]),
        ("share/" + package_name + "/config",
         ["config/eye_in_hand_result.json", "config/eye_in_hand_result.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="qluo",
    maintainer_email="qluo@example.com",
    description="xArm6 OAK-D-SR eye-in-hand calibration tools",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "collect_eye_in_hand = xarm_oak_handeye.collect_eye_in_hand:main",
            "auto_eye_in_hand = xarm_oak_handeye.auto_eye_in_hand:main",
            "test_apriltag_board = xarm_oak_handeye.test_apriltag_board:main",
            "test_depthai_apriltag_board = xarm_oak_handeye.test_depthai_apriltag_board:main",
            "solve_eye_in_hand = xarm_oak_handeye.solve_eye_in_hand:main",
            "publish_handeye_tf = xarm_oak_handeye.publish_handeye_tf:main",
            "target_to_base = xarm_oak_handeye.target_to_base:main",
            "oak_peach_detector = xarm_oak_handeye.oak_peach_detector:main",
            "check_handeye_scatter = xarm_oak_handeye.check_handeye_scatter:main",
            "launch_moveit_with_oak = xarm_oak_handeye.launch_moveit_with_oak:main",
        ],
    },
)
