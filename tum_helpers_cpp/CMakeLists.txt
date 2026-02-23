cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tum_helpers_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED 
  src/curvilinear_cosy.cpp
  src/segment.cpp
  src/delay_compensation.cpp
  src/vehicle_dynamics.cpp
  src/tire_models.cpp
  src/ccma.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}/include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
# target_compile_options(${PROJECT_NAME} PRIVATE -Ofast -fopenmp) #-dndebug -fno-math-errno -fopenmp)
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
ament_target_dependencies(${PROJECT_NAME} Eigen3 tum_types_cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

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

# ament_target_dependencies(${PROJECT_NAME} INTERFACE)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(Eigen3 tum_types_cpp) # other dependencies so client does not need to call find_package on them

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(cosy_tests test/curvilinear_cosy_tests.cpp)
  target_link_libraries(cosy_tests ${PROJECT_NAME}) # make our library available to the test executable

  ament_add_gtest(geometry_tests test/geometry_helper_tests.cpp)
  target_link_libraries(geometry_tests ${PROJECT_NAME}) # make our library available to the test executable
  
  ament_add_gtest(numerical_test test/numerical_helper_tests.cpp)
  target_link_libraries(numerical_test ${PROJECT_NAME}) # make our library available to the test executable

  ament_add_gtest(delay_compensation_test test/delay_compensation_test.cpp)
  target_link_libraries(delay_compensation_test ${PROJECT_NAME}) # make our library available to the test executable

  add_subdirectory(test)
endif()

target_compile_definitions(tum_helpers_cpp
  PRIVATE BUILDING_CURVILINEAR_COSY
)


ament_package()