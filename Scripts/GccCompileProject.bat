cd ../
cmake --no-warn-unused-cli -DCMAKE_MAKE_PROGRAM:STRING=C:\msys64\usr\bin\make.exe -DCMAKE_C_COMPILER:FILEPATH=C:\msys64\mingw64\bin\gcc.exe -DCMAKE_CXX_COMPILER:FILEPATH=C:\msys64\mingw64\bin\g++.exe -B build -G "Unix Makefiles"
"C:\Program Files\CMake\bin\cmake.EXE" --build build --target all -j 14 --
PAUSE