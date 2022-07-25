cmake_minimum_required(VERSION 3.10)  

project(DEMO)

include_directories(".")

aux_source_directory(. SRCS)

add_executable(demo ${SRCS})

target_link_libraries(demo pthread)