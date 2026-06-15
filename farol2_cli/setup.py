from setuptools import find_packages, setup

package_name = "farol2_cli"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/shell", ["shell/farol2.bash"]),
    ],
    install_requires=["setuptools", "argcomplete"],
    zip_safe=True,
    maintainer="Ravi Regalo",
    maintainer_email="raviregalo@gmail.com",
    description="Command-line helpers for FAROL2 development workflows.",
    license="MIT",
    entry_points={"console_scripts": ["farol2 = farol2_cli.main:main"]},
)
