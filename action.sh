cd build
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPYTHON_EXECUTABLE=$(which python3)
make -j16
