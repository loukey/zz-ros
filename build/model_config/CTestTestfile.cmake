# CMake generated Testfile for 
# Source directory: /home/za/Codes/zzros/src/model_config
# Build directory: /home/za/Codes/zzros/build/model_config
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(flake8 "/home/za/anaconda3/envs/ros/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/za/Codes/zzros/build/model_config/test_results/model_config/flake8.xunit.xml" "--package-name" "model_config" "--output-file" "/home/za/Codes/zzros/build/model_config/ament_flake8/flake8.txt" "--command" "/opt/ros/jazzy/bin/ament_flake8" "--xunit-file" "/home/za/Codes/zzros/build/model_config/test_results/model_config/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/za/Codes/zzros/src/model_config" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_flake8/cmake/ament_flake8.cmake;69;ament_add_test;/opt/ros/jazzy/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;19;ament_flake8;/opt/ros/jazzy/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;31;ament_package;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;0;")
add_test(lint_cmake "/home/za/anaconda3/envs/ros/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/za/Codes/zzros/build/model_config/test_results/model_config/lint_cmake.xunit.xml" "--package-name" "model_config" "--output-file" "/home/za/Codes/zzros/build/model_config/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/jazzy/bin/ament_lint_cmake" "--xunit-file" "/home/za/Codes/zzros/build/model_config/test_results/model_config/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/za/Codes/zzros/src/model_config" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;31;ament_package;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;0;")
add_test(pep257 "/home/za/anaconda3/envs/ros/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/za/Codes/zzros/build/model_config/test_results/model_config/pep257.xunit.xml" "--package-name" "model_config" "--output-file" "/home/za/Codes/zzros/build/model_config/ament_pep257/pep257.txt" "--command" "/opt/ros/jazzy/bin/ament_pep257" "--xunit-file" "/home/za/Codes/zzros/build/model_config/test_results/model_config/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/za/Codes/zzros/src/model_config" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/opt/ros/jazzy/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/opt/ros/jazzy/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;31;ament_package;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;0;")
add_test(xmllint "/home/za/anaconda3/envs/ros/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/za/Codes/zzros/build/model_config/test_results/model_config/xmllint.xunit.xml" "--package-name" "model_config" "--output-file" "/home/za/Codes/zzros/build/model_config/ament_xmllint/xmllint.txt" "--command" "/opt/ros/jazzy/bin/ament_xmllint" "--xunit-file" "/home/za/Codes/zzros/build/model_config/test_results/model_config/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/za/Codes/zzros/src/model_config" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;31;ament_package;/home/za/Codes/zzros/src/model_config/CMakeLists.txt;0;")
