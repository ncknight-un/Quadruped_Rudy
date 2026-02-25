# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(rudylib)

# Find dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
find_package(Doxygen)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target" (in this case the target is called "libname"
add_library(rudylib
    src/LegKinematics.cpp
    src/RudyKinematics.cpp
)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# using the library from the build directory or after installation the header
# files are actually in different locations.
# During the build, the headers are read from the source code directory
# When used from the installed location, headers are in the system include/ directory
target_include_directories(rudylib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
    ${EIGEN3_INCLUDE_DIRS}
)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(rudylib PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support.
# PUBLIC causes the features to propagate to anything
# that links against this library
# whereas PRIVATE would make it apply only to the library itself
target_compile_features(rudylib PUBLIC cxx_std_23)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to the locations of header files.
# Because libname has PUBLIC cxx_std_23 as a compile feature
# Name will also be compiled with c++23 as the standard.
#target_link_libraries(foot_position PRIVATE rudylib Eigen3::Eigen)

# install the include files by copying the whole include directory
install(DIRECTORY include/rudylib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called project_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
# install(TARGETS rudylib foot_position EXPORT rudylib-targets)  ///#####

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
# install(EXPORT rudylib-targets
#         FILE rudylib-config.cmake
#         NAMESPACE rudylib::
#         DESTINATION share/${PROJECT_NAME}/cmake)

# THIS Doxygen formater has been removed due to compiling issue, but is an example of how to set up.
# # Building documentation should be optional.
# # To build documentation pass -DBUILD_DOCS=ON when generating the build system
# option(BUILD_DOCS "Build the documentation" ON)

# # build the documentation
# if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
#     # Turn the README.md into the homepage of the doxygen docs
#     set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

#     # Tell Doxygen where to find the documentation
#     doxygen_add_docs(doxygen include/ src/ README.md ALL)

#     # The documentation will be in the build/html directory
#     # The main page is build/html/index.html
# endif()

# Execution of Unit Testing CTests:  (run ctest --verbose)
include(CTest)

option(BUILD_TESTING "Allow Unit Test Execution" ON)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)

    # A test is just an executable that is linked against the unit testing library
    add_executable(test_foot tests/leg_kinematics_test.cpp)
    target_link_libraries(test_foot Catch2::Catch2WithMain rudylib Eigen3::Eigen)

    add_executable(test_body tests/body_kinematics_test.cpp)
    target_link_libraries(test_body Catch2::Catch2WithMain rudylib Eigen3::Eigen)

    # register the test with CTest, telling it what executable to run
    add_test(NAME TestFoot COMMAND test_foot)
    add_test(NAME TestBody COMMAND test_body)
endif()