# CMake generated Testfile for 
# Source directory: /home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot
# Build directory: /home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/home/ajak2k/ros2_humble/ros2/share/ament_cmake_test/cmake/run_test.py" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/test_results/TurtleBot/lint_cmake.xunit.xml" "--package-name" "TurtleBot" "--output-file" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/ament_lint_cmake/lint_cmake.txt" "--command" "/home/ajak2k/ros2_humble/ros2/bin/ament_lint_cmake" "--xunit-file" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/test_results/TurtleBot/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot" _BACKTRACE_TRIPLES "/home/ajak2k/ros2_humble/ros2/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/ajak2k/ros2_humble/ros2/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/ajak2k/ros2_humble/ros2/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot/CMakeLists.txt;26;ament_package;/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/home/ajak2k/ros2_humble/ros2/share/ament_cmake_test/cmake/run_test.py" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/test_results/TurtleBot/xmllint.xunit.xml" "--package-name" "TurtleBot" "--output-file" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/ament_xmllint/xmllint.txt" "--command" "/home/ajak2k/ros2_humble/ros2/bin/ament_xmllint" "--xunit-file" "/home/ajak2k/Sriram_AjayShankar_ws/build/TurtleBot/test_results/TurtleBot/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot" _BACKTRACE_TRIPLES "/home/ajak2k/ros2_humble/ros2/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/ajak2k/ros2_humble/ros2/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/ajak2k/ros2_humble/ros2/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/ajak2k/ros2_humble/ros2/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot/CMakeLists.txt;26;ament_package;/home/ajak2k/Sriram_AjayShankar_ws/src/TurtleBot/CMakeLists.txt;0;")
