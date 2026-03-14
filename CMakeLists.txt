cmake_minimum_required(VERSION 3.18)
project(hav_engine VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ─── Compiler flags (MSVC vs GCC/Clang) ──────────────────────────────────────
if(MSVC)
  add_compile_options(/O2 /W3 /wd4996 /MP)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0")
  add_compile_options(-Wall -Wextra)
endif()

# ─── Find pybind11 ────────────────────────────────────────────────────────────
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.12.0
  )
  FetchContent_MakeAvailable(pybind11)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# ─── Core static library ──────────────────────────────────────────────────────
add_library(hav_core STATIC core/hav_engine.cpp)
target_include_directories(hav_core PUBLIC core/)

# ─── pybind11 Python module ───────────────────────────────────────────────────
pybind11_add_module(hav_native core/bindings.cpp)
target_link_libraries(hav_native PRIVATE hav_core)
target_include_directories(hav_native PRIVATE core/)

# On Windows the .pyd goes next to hav_api.py so Python can import it
set_target_properties(hav_native PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY                 "${CMAKE_SOURCE_DIR}/core"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE         "${CMAKE_SOURCE_DIR}/core"
  LIBRARY_OUTPUT_DIRECTORY_DEBUG           "${CMAKE_SOURCE_DIR}/core"
  RUNTIME_OUTPUT_DIRECTORY                 "${CMAKE_SOURCE_DIR}/core"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE         "${CMAKE_SOURCE_DIR}/core"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG           "${CMAKE_SOURCE_DIR}/core"
)

# ─── CLI benchmark binary ─────────────────────────────────────────────────────
add_executable(hav_cli cli/main_cli.cpp)
target_link_libraries(hav_cli PRIVATE hav_core)
target_include_directories(hav_cli PRIVATE core/)

set_target_properties(hav_cli PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_SOURCE_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_SOURCE_DIR}"
)

install(TARGETS hav_native DESTINATION ${Python3_SITEARCH})
