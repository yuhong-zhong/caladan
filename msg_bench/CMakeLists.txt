cmake_minimum_required(VERSION 3.10)
project(msg_bench)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(/usr/local/include)

add_compile_options(-mwaitpkg)
add_compile_options(-march=native)
add_compile_options(-mavx512f)

add_executable(msg_bench msg_bench.cpp)

target_link_libraries(msg_bench numa)
