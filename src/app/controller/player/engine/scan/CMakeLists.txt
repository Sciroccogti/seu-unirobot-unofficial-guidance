add_library(scan STATIC
        ScanEngine.cpp)

target_link_libraries(scan pthread)  