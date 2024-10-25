cmake_minimum_required(VERSION 3.18)
project(MandelbrotLibrary LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)
cmake_policy(SET CMP0146 NEW)
find_package(CUDAToolkit REQUIRED)
find_package(JNI REQUIRED)

add_library(MandelbrotLibrary SHARED
        src/httplib.h
        src/kernel.cu
        src/kernel.cuh
        src/MandelbrotJNI.cpp
        src/stb_image_write.h
        src/kernel.cuh
        src/MandelbrotJNI.cpp
)

if (WIN32)
    target_compile_definitions(MandelbrotLibrary PRIVATE MANDELBROT_LIBRARY_EXPORTS)
endif ()

include_directories(src)
include_directories(${JNI_INCLUDE_DIRS})
target_link_libraries(MandelbrotLibrary ${CUDA_LIBRARIES} ${JNI_LIBRARIES})

set_target_properties(MandelbrotLibrary PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_STANDARD 14
)