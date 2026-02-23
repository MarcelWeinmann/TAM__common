cmake_minimum_required(VERSION 3.5)
project(track_handler_py)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Portable arch flags (list elements, not a single string)
set(SP_ARCH_OPTS
  # $<$<NOT:$<CONFIG:Debug>>:-march=x86-64-v3>
  # $<$<NOT:$<CONFIG:Debug>>:-mtune=znver3>
)

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

# set(SP_DEBUG_OPTS
#   -O0
#   -g3
#   -fno-omit-frame-pointer
# )

# target_compile_options(${PROJECT_NAME} PRIVATE -O3 -fopenmp) #-dndebug -fno-math-errno -fopenmp)
# target_compile_options(${PROJECT_NAME} PRIVATE ${SP_ARCH_OPTS} ${SP_FAST_OPTS} ${SP_DEBUG_OPTS})

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
#find_package(Eigen3 REQUIRED)

find_package(track_handler_cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_cpp_binding SHARED
  src/${PROJECT_NAME}/_cpp_binding.cpp
)

target_compile_options(_cpp_binding PRIVATE
  ${SP_ARCH_OPTS} ${SP_FAST_OPTS} ${SP_DEBUG_OPTS}
)

# ament_target_dependencies(_reindexer PUBLIC
#   "rosbag2_cpp"
#   "rosbag2_storage"
# )
target_include_directories(_cpp_binding
    PUBLIC
        #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        #$<INSTALL_INTERFACE:include>
        /usr/include/eigen3
)


ament_target_dependencies(_cpp_binding PUBLIC
  track_handler_cpp
#  Eigen3
)

# Install cython modules as sub-modules of the project
install(
  TARGETS
    _cpp_binding
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   fnt_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(Eigen3 tum_types_cpp) # Work around a rtti bug when using class_loader from a
#     # python extension built with libc++.
#     set(set_env_vars "${set_env_vars} pybind11_demo_TEST_WITH_RTLD_GLOBAL=True")
#   endif()

#   # message(FATAL_ERROR "HELP I AM THE PATH ${CMAKE_CURRENT_SOURCE_DIR}")
#   # ament_add_pytest_test(test_sequential_reader_py "test/test_sequential_reader.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
#   # ament_add_pytest_test(test_sequential_reader_multiple_files_py "test/test_sequential_reader_multiple_files.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
#   # ament_add_pytest_test(test_sequential_writer_py
#   #   "test/test_sequential_writer.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
#   # ament_add_pytest_test(test_transport_py
#   #   "test/test_transport.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
#   # ament_add_pytest_test(test_reindexer_py
#   #   "test/test_reindexer.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
#   # ament_add_pytest_test(test_convert_py
#   #   "test/test_convert.py"
#   #   APPEND_ENV "${append_env_vars}"
#   #   ENV "${set_env_vars}"
#   # )
# endif()

ament_package()
