cmake_minimum_required(VERSION 3.22)
project(turtlelib)

# Create libraries
add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/diff_drive.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
target_compile_features(turtlelib PUBLIC cxx_std_17)

# Cmake option for build testing off
option(BUILD_TESTING "Build the unit tests" ON)

# Catch2 unit testing framework
if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)
  include(CTest)
  enable_testing()
  add_executable(geometry2d_test tests/test_geometry2d.cpp)
  add_executable(se2d_test tests/test_se2d.cpp)
  add_executable(svg_test tests/test_svg.cpp)
  add_executable(diff_drive_test tests/test_diff_drive.cpp)
  target_link_libraries(geometry2d_test turtlelib Catch2::Catch2WithMain)
  target_link_libraries(se2d_test turtlelib Catch2::Catch2WithMain)
  target_link_libraries(svg_test turtlelib Catch2::Catch2WithMain)
  target_link_libraries(diff_drive_test turtlelib Catch2::Catch2WithMain)
  add_test(NAME Test_Geometry2D COMMAND geometry2d_test)
  add_test(NAME Test_SE2D COMMAND se2d_test)
  add_test(NAME Test_Svg COMMAND svg_test)
  add_test(NAME Test_DiffDrive COMMAND diff_drive_test)
endif()

# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# create the executable target and link with turtlelib 
add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS frame_main turtlelib EXPORT turtlelib-targets)

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