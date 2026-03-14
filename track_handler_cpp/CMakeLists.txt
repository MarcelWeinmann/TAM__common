cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME track_handler_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(param_management_cpp REQUIRED)
find_package(param_management_ros2_integration_cpp REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/track.cpp
  src/raceline.cpp
  src/race_track_handler.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${YAML_INCLUDE_DIRS}>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
# target_compile_options(${PROJECT_NAME} PRIVATE -O3 -fopenmp) #-dndebug -fno-math-errno -fopenmp)

# Aggressive non-Debug opts (one genex per flag)
# set(SP_FAST_OPTS
#   $<$<NOT:$<CONFIG:Debug>>:-Ofast>
#   $<$<NOT:$<CONFIG:Debug>>:-ffast-math>
#   $<$<NOT:$<CONFIG:Debug>>:-fno-math-errno>
#   $<$<NOT:$<CONFIG:Debug>>:-fno-trapping-math>
#   $<$<NOT:$<CONFIG:Debug>>:-ffp-contract=fast>
#   $<$<NOT:$<CONFIG:Debug>>:-finline-functions>
#   $<$<NOT:$<CONFIG:Debug>>:-funroll-loops>
#   $<$<NOT:$<CONFIG:Debug>>:-fpeel-loops>
#   $<$<NOT:$<CONFIG:Debug>>:-funswitch-loops>
#   $<$<NOT:$<CONFIG:Debug>>:-ftree-vectorize>
#   $<$<NOT:$<CONFIG:Debug>>:-fvect-cost-model=very-cheap>
#   $<$<NOT:$<CONFIG:Debug>>:-fomit-frame-pointer>
#   # Per-compiler extras
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:GNU>>:-frename-registers>
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:GNU>>:-fira-loop-pressure>
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:Clang>>:-mllvm>
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:Clang>>:-inline-threshold=1000>
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:Clang>>:-mllvm>
#   $<$<AND:$<NOT:$<CONFIG:Debug>>,$<CXX_COMPILER_ID:Clang>>:-unroll-threshold=1000>
# )

# Portable arch flags (list elements, not a single string)
set(SP_ARCH_OPTS
  # $<$<NOT:$<CONFIG:Debug>>:-march=x86-64-v3>
  # $<$<NOT:$<CONFIG:Debug>>:-mtune=znver3>
)

set(SP_FAST_OPTS
        $<$<NOT:$<CONFIG:Debug>>:-O3>
        $<$<NOT:$<CONFIG:Debug>>:-DNDEBUG>
        )

# Optional Debug niceties (again: list, no quotes)
set(SP_DEBUG_OPTS
  $<$<CONFIG:Debug>:-O0>
  $<$<CONFIG:Debug>:-g3>
  $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
)

target_compile_options(${PROJECT_NAME} PRIVATE ${SP_ARCH_OPTS} ${SP_FAST_OPTS} ${SP_DEBUG_OPTS})

ament_target_dependencies(${PROJECT_NAME} PUBLIC Eigen3 ament_index_cpp param_management_cpp param_management_ros2_integration_cpp tum_helpers_cpp tum_types_cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${YAML_CPP_LIBRARIES} )

install(
  DIRECTORY config_overwrite
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(Eigen3 ament_index_cpp tum_helpers_cpp param_management_cpp param_management_ros2_integration_cpp) # other dependencies so client does not need to call find_package on them

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(track_handler_tests test/track_handler_tests.cpp)
  target_link_libraries(track_handler_tests ${PROJECT_NAME}) # make our library available to the test executable
  
  ament_add_gtest(acceleration_tests test/acceleration_tests.cpp)
  target_link_libraries(acceleration_tests ${PROJECT_NAME})
  
  ament_add_gtest(track_helpers_tests test/track_helpers_tests.cpp)
  target_link_libraries(track_helpers_tests ${PROJECT_NAME})

  ament_add_gtest(rotation_tests test/rotation_tests.cpp)
  target_link_libraries(rotation_tests ${PROJECT_NAME})
endif()

add_subdirectory(test)

target_compile_definitions(track_handler_cpp
  PRIVATE BUILDING_CURVILINEAR_COSY
)

ament_package()

# # Create the executable
# add_executable(module test/module.cpp)
# # Link against the lib and private dependencies
# target_link_libraries(module ${PROJECT_NAME}) 
# # Install target
# install(
#   TARGETS module  
#   DESTINATION test/${PROJECT_NAME}
# )
