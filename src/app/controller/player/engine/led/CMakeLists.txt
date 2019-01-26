add_library(led STATIC
        LedEngine.cpp)

target_link_libraries(led pthread)  