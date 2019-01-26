add_library(action STATIC
        ActionEngine.cpp)

target_link_libraries(action pthread)  