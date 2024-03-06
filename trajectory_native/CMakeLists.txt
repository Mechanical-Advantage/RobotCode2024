cmake_minimum_required(VERSION 3.24)
project(trajectory_native)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--whole-archive --allow-multiple-definition")

find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(TrajoptLib REQUIRED)

add_library(trajectory_native_proto proto/VehicleTrajectoryService.proto)
target_link_libraries(trajectory_native_proto PUBLIC protobuf::libprotobuf gRPC::grpc gRPC::grpc++ gRPC::grpc++_reflection)
target_include_directories(trajectory_native_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET trajectory_native_proto LANGUAGE cpp)
protobuf_generate(TARGET trajectory_native_proto LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")


add_executable(trajectory_native src/trajectory_service.cpp)
target_link_libraries(trajectory_native PRIVATE TrajoptLib::TrajoptLib trajectory_native_proto fmt::fmt nlohmann_json::nlohmann_json)
