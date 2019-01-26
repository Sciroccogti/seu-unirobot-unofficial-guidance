file(GLOB CFILES "*.c")
file(GLOB CUFILES "*.cu")

add_library(darknet STATIC ${CUFILES} ${CFILES})
target_link_libraries(darknet 
    cudnn 
    cudart 
    cublas 
    curand)
