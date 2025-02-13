cmake_minimum_required(VERSION 3.16)
project(Moccer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Mac's Protobuf path settings
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(Protobuf_INCLUDE_DIR "/opt/homebrew/opt/protobuf@21/include")
        set(Protobuf_LIBRARIES "/opt/homebrew/opt/protobuf@21/lib/libprotobuf.dylib")
        include_directories("/opt/homebrew/opt/protobuf@21/include")
    else()
        set(Protobuf_INCLUDE_DIR "/usr/local/Cellar/protobuf@21/21.12_1/include")
        set(Protobuf_LIBRARIES "/usr/local/Cellar/protobuf@21/21.12_1/lib/libprotobuf.dylib")
        include_directories("/usr/local/Cellar/protobuf@21/21.12_1/include")
    endif()
endif()

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
# set(MODEL_DIR ${SRC_DIR}/models)
set(NET_DIR ${SRC_DIR}/networks)

# file(GLOB SOURCES ${SRC_DIR}/*.cpp)
# file(GLOB MODEL_SOURCES ${MODEL_DIR}/*.cpp)
file(GLOB NET_SOURCES ${NET_DIR}/*.cpp)

# file(GLOB HEADERS ${SRC_DIR}/*.h)
# file(GLOB MODEL_HEADERS ${MODEL_DIR}/*.h)
file(GLOB NET_HEADERS ${NET_DIR}/*.h)



# Compile Protobuf files
file(GLOB PROTO_FILES "${PROTOBUF_SOURCE_DIR}/*.proto")
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
    ${NET_SOURCES}
)

include_directories(${PROTOBUF_GENERATE_DIR})

# Executable settings
add_executable(Moccer main.cpp ${SOURCES})

# Link libraries
target_link_libraries(Moccer
    ${Protobuf_LIBRARIES}
)

# Set runtime directory
set_target_properties(Moccer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Custom 'run' target
add_custom_target(run
    COMMAND ${CMAKE_BINARY_DIR}/bin/Moccer
    DEPENDS Moccer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
