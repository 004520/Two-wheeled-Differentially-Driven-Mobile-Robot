from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mycar_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mycar",
    maintainer_email="mycar@todo.todo",
    description="YOLO11n + classical colour vision front end for mycar.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_detector = mycar_vision.vision_detector:main",
            "vision_radar_fusion = mycar_vision.vision_radar_fusion:main",
            "avoidance_decider = mycar_vision.avoidance_decider:main",
            "vision_to_costmap = mycar_vision.vision_to_costmap:main",
            "fake_camera_publisher = mycar_vision.fake_camera_publisher:main",
            "vision_web_stream = mycar_vision.vision_web_stream:main",
        ],
    },
)
