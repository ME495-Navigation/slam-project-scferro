cmake_minimum_required(VERSION 3.22)
project(turtlelib)

# Create libraries
add_library(turtlelib src/geometry2d.cpp )

# Use target_include_directories so that #include"mylibrary/header.hpp" works
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
target_compile_features(turtlelib PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"

# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
install(TARGETS Name turtlelib EXPORT turtlelib-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})