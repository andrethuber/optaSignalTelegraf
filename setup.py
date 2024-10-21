from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension(
        "library",
        ["tests/library_wrapper.cpp", "main/library.c"],
    ),
]

setup(
    name="lib",
    version="1.0",
    author="Your Name",
    description="A simple C library wrapped with pybind11",
    long_description="",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
