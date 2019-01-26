add_library(tcp_client STATIC
            tcp_client.cpp)

target_link_libraries(tcp_client
            boost_system
            pthread)