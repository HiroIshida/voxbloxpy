#!/bin/bash
find . -name "*.py" ! -path "*/pybind11/*" ! -path "*/voxblox/*" | xargs -i{} python3 -m autoflake -i --remove-all-unused-imports --remove-unused-variables --ignore-init-module-imports {}
python3 -m isort ./python ./example
python3 -m black ./python ./example --config .pyproject.toml
python3 -m flake8 ./python ./example
