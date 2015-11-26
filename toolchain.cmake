SET(CMAKE_SYSTEM_NAME Linux)  # Tell CMake we're cross-compiling
include(CMakeForceCompiler)

# CMake will look for prefixed g++, cpp, ld, etc. automatically
CMAKE_FORCE_C_COMPILER(arm-linux-gnueabihf-gcc-4.8 GNU)
CMAKE_FORCE_CXX_COMPILER(arm-linux-gnueabihf-g++-4.8 GNU)

SET(CMAKE_FIND_ROOT_PATH  /usr/arm-linux-gnueabi)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
