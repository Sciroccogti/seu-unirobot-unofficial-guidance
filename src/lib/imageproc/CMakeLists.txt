add_library(imageproc
        imageproc.cpp)

target_link_libraries(imageproc
        ${OpenCV_LIBS})
