add_subdirectory(QuinticWalk)
add_library(walk STATIC
            WalkEngine.cpp)

target_link_libraries(walk pthread quinticwalk)
