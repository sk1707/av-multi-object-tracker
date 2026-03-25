# CMake generated Testfile for 
# Source directory: /Users/bapnarb/Desktop/kalman-filter-cpp
# Build directory: /Users/bapnarb/Desktop/kalman-filter-cpp/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(EKFTests "/Users/bapnarb/Desktop/kalman-filter-cpp/build/run_tests")
set_tests_properties(EKFTests PROPERTIES  _BACKTRACE_TRIPLES "/Users/bapnarb/Desktop/kalman-filter-cpp/CMakeLists.txt;29;add_test;/Users/bapnarb/Desktop/kalman-filter-cpp/CMakeLists.txt;0;")
subdirs("_deps/googletest-build")
