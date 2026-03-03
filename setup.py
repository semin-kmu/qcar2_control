from setuptools import find_packages, setup

package_name = "qcar2_control"

setup(
    name=package_name,
    version="0.9.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    package_data={
        "qcar2_control.scalecar_vfg.data.controllers": ["*.json"],
    },
    zip_safe=True,
    maintainer="jeon_semin",
    maintainer_email="eiffeltower1206@kookmin.ac.kr",
    description="ACC control nodes",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bezier_path_publisher = qcar2_control.bezier_path_publisher:main",
            "ph_path_publisher = qcar2_control.ph_path_publisher:main",
            "dashboard = qcar2_control.dashboard:main",
            "scalecar_hinf_path_follower = qcar2_control.scalecar_hinf_path_follower:main",
        ],
    },
)
