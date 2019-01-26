add_library(debuger_options STATIC
            options.cpp)

target_link_libraries(debuger_options
            boost_program_options)