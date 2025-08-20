cmake_minimum_required(VERSION 3.16)
project(Moccer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Mac's Protobuf path settings
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(Protobuf_INCLUDE_DIR "/opt/homebrew/opt/protobuf@21/include")
    set(Protobuf_LIBRARIES "/opt/homebrew/opt/protobuf@21/lib/libprotobuf.dylib")
    include_directories("/opt/homebrew/opt/protobuf@21/include")
    include_directories("/opt/homebrew/opt/eigen/include")
    include_directories("/opt/homebrew/opt/yaml-cpp/include")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux Protobuf path settings
    set(Protobuf_INCLUDE_DIR "/usr/local/include")
    set(Protobuf_LIBRARIES "/usr/local/lib/libprotobuf.a")
    include_directories("/usr/local/include")
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

find_package(Threads REQUIRED)

# if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
#     cmake_policy(SET CMP0167 OLD)
# endif()
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Boost 1.70 REQUIRED)
# target_link_libraries(Moccer Boost::Boost)
include_directories(${Boost_INCLUDE_DIRS})

# Protobuf settings
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf 3.21.12 REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

# Protobuf directories
set(PROTOBUF_DIR ${CMAKE_SOURCE_DIR}/proto)
set(PROTOBUF_SOURCE_DIR ${PROTOBUF_DIR}/pb_src)
set(PROTOBUF_GENERATE_DIR ${PROTOBUF_DIR}/pb_gen)

file(MAKE_DIRECTORY ${PROTOBUF_GENERATE_DIR})

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(MODEL_DIR ${SRC_DIR}/models)
set(NET_DIR ${SRC_DIR}/networks)

file(GLOB MAIN_SOURCES ${SRC_DIR}/*.cpp)
file(GLOB MODEL_SOURCES ${MODEL_DIR}/*.cpp)
file(GLOB NET_SOURCES ${NET_DIR}/*.cpp)
file(GLOB FINDER_SOURCES ${SRC_DIR}/planner/*.cpp)

# file(GLOB HEADERS ${SRC_DIR}/*.h)
# file(GLOB MODEL_HEADERS ${MODEL_DIR}/*.h)
# file(GLOB NET_HEADERS ${NET_DIR}/*.h)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Compile Protobuf files
file(GLOB PROTO_FILES "${PROTOBUF_SOURCE_DIR}/*.proto")

# Yaml files
file(GLOB YAML_FILES "${CMAKE_SOURCE_DIR}/config/*.yaml")

protobuf_generate_cpp(
    PROTO_SRCS 
    PROTO_HDRS 
    ${PROTO_FILES} 
    IMPORT_DIRS 
    ${PROTOBUF_SOURCE_DIR} 
    PROTOC_OUT_DIR 
    ${PROTOBUF_GENERATE_DIR}
)

# Source files
set(SOURCES
    ${PROTO_SRCS}
    ${YAML_FILES}
    ${MAIN_SOURCES}
    ${MODEL_SOURCES}
    ${NET_SOURCES}
    ${FINDER_SOURCES}
)

include_directories(${PROTOBUF_GENERATE_DIR})

# Executable settings
add_executable(Moccer main.cpp ${SOURCES})

# Link libraries

target_link_libraries(Moccer
    ${Protobuf_LIBRARIES}
    Eigen3::Eigen
    Threads::Threads
    yaml-cpp
)


# Set runtime directory
set_target_properties(Moccer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Custom 'run' target
add_custom_target(run
    COMMAND ${CMAKE_BINARY_DIR}/bin/Moccer
    DEPENDS Moccer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
