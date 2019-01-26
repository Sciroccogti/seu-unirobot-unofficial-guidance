cmake_minimum_required(VERSION 3.12)

function(detect_installed_gpu out_variable)
    set(__cufile "${CMAKE_BINARY_DIR}/detect_cuda_archs.cu")
    file(WRITE ${__cufile} ""
            "#include <cstdio>\n"
            "int main()\n"
            "{\n"
            "  int count = 0;\n"
            "  if (cudaSuccess != cudaGetDeviceCount(&count)) return -1;\n"
            "  if (count == 0) return -1;\n"
            "  cudaDeviceProp prop;\n"
            "  if (cudaSuccess == cudaGetDeviceProperties(&prop, 0))\n"
            "    std::printf(\"%d%d \", prop.major, prop.minor);\n"
            "  else std::printf(\"50\");\n"
            "  return 0;\n"
            "}\n")
    execute_process(COMMAND "/usr/local/cuda/bin/nvcc" "--run" "${__cufile}"
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/CMakeFiles/"
            RESULT_VARIABLE __nvcc_res OUTPUT_VARIABLE __nvcc_out
            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(__nvcc_res EQUAL 0)
        set(${out_variable} "${__nvcc_out}" PARENT_SCOPE)
    else()
        set(${out_variable} "50" PARENT_SCOPE)
    endif()
endfunction()

if("${CROSS}" STREQUAL "ON")
    set(CMAKE_SYSTEM_NAME "Linux")
    set(CMAKE_SYSTEM_PROCESSOR "aarch64")
    set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
    set(CMAKE_CUDA_FLAGS "-gencode arch=compute_62,code=[sm_62,compute_62] -ccbin aarch64-linux-gnu-g++ -m64")
    set(TX2_LIBS "$ENV{HOME}/JetsonTX2")
    include_directories("/usr/local/cuda/targets/aarch64-linux/include"
                        "${TX2_LIBS}/include")
    link_directories("${TX2_LIBS}/lib")
else()
    detect_installed_gpu(cuda_arch)
    set(CMAKE_CUDA_FLAGS "-gencode arch=compute_${cuda_arch},code=[sm_${cuda_arch},compute_${cuda_arch}]")
    include_directories("/usr/local/cuda/include")
endif()

project(seu-unirobot LANGUAGES C CXX CUDA)

message(STATUS "Target platform: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CUDA_FLAGS: ${CMAKE_CUDA_FLAGS}")

add_definitions(-DGPU)
add_definitions(-DCUDNN)
add_definitions(-DCUDNN_HALF)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")

add_subdirectory(src)
