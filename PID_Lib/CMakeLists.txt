cmake_minimum_required(VERSION 3.15)

project(Lib)
set(CMAKE_CXX_STANDARD 20)

# Create library
add_library(PID_Lib STATIC
    src/PID_Controller.cpp)

target_include_directories(PID_Lib PUBLIC inc)

# Create an executable for the Catch2 test code
add_executable(PID_Test 
    test/test_main.cpp
    test/test_PID_Controller.cpp)

# Add the source files to the test executable
target_link_libraries(PID_Test PUBLIC PID_Lib)

# Link Catch2 with the test executable
target_link_libraries(PID_Test PRIVATE Catch2::Catch2)