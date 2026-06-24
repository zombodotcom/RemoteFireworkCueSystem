cmake_minimum_required(VERSION 3.16)
project(cyd_host_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../main
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/fireworkcore/include)
add_executable(test_display_decode test_display_decode.cpp ../main/display_decode.cpp)
add_test(NAME test_display_decode COMMAND test_display_decode)
