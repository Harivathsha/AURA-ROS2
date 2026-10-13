cmake_minimum_required(VERSION 3.8)
project(aura_semantic_slam)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/semantic_slam_mapper.py scripts/rgbd_adapter.py
        DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/semantic_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(semantic_slam_core
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_slam.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)
endif()
ament_package()
