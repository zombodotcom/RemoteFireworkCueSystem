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
    ${CORE}/src/arming.cpp
    ${CORE}/src/sequence.cpp
    ${CORE}/src/box_controller.cpp
    ${CORE}/src/box_link.cpp
    ${CORE}/src/show_runner.cpp
)

add_executable(test_crc32 test_crc32.cpp ${CORE_SRC})
add_test(NAME test_crc32 COMMAND test_crc32)

add_executable(test_protocol test_protocol.cpp ${CORE_SRC})
add_test(NAME test_protocol COMMAND test_protocol)

add_executable(test_recent_ids test_recent_ids.cpp ${CORE_SRC})
add_test(NAME test_recent_ids COMMAND test_recent_ids)

add_executable(test_arming test_arming.cpp ${CORE_SRC})
add_test(NAME test_arming COMMAND test_arming)

add_executable(test_sequence test_sequence.cpp ${CORE_SRC})
add_test(NAME test_sequence COMMAND test_sequence)

add_executable(test_rig test_rig.cpp ${CORE_SRC} ${CORE}/wasm/sim_bindings.cpp)
target_include_directories(test_rig PRIVATE ${CORE}/wasm)
add_test(NAME test_rig COMMAND test_rig)

add_executable(test_box_controller test_box_controller.cpp ${CORE_SRC})
add_test(NAME test_box_controller COMMAND test_box_controller)

add_executable(test_box_link test_box_link.cpp ${CORE_SRC})
add_test(NAME test_box_link COMMAND test_box_link)

add_executable(test_show_runner test_show_runner.cpp ${CORE_SRC})
add_test(NAME test_show_runner COMMAND test_show_runner)
