cmake_minimum_required(VERSION 3.16)
project(PBS)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 17)

include_directories(inc)
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost 1.78.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories(
        ${Boost_INCLUDE_DIRS}
        "inc"
)

add_executable(PBS ${SOURCES})

target_link_libraries(PBS ${Boost_LIBRARIES})
