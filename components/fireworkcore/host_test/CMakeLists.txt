cmake_minimum_required(VERSION 3.16)
project(fireworkcore_host_test CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CORE}/include ${CMAKE_CURRENT_SOURCE_DIR})

# Pure-core sources linked into every test (harmless if unused by a given test).
set(CORE_SRC
    ${CORE}/src/crc32.cpp
)

add_executable(test_crc32 test_crc32.cpp ${CORE_SRC})
add_test(NAME test_crc32 COMMAND test_crc32)

add_executable(test_protocol test_protocol.cpp ${CORE_SRC})
add_test(NAME test_protocol COMMAND test_protocol)
