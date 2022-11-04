from skbuild import setup

install_requires = [
    "numpy",
    "plotly",
    "scipy"
]

setup(
    name="voxbloxpy",
    version="0.0.2",
    description="standalone voxblox python",
    author="Hirokazu Ishida",
    license="MIT",
    packages=["voxbloxpy"],
    package_dir={"": "python"},
    cmake_install_dir="python/voxbloxpy/",
    install_requires=install_requires,
    package_data={"voxbloxpy": ["py.typed"]},
)
