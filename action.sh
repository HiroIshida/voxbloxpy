cd build
cmake .. -DBUILD_TEST=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPYTHON_EXECUTABLE=$(which python3)
if [ $? == 0 ]; then
    make -j16
fi
if [ $? == 0 ]; then
    make install
fi
