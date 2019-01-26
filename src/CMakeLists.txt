include_directories(lib)
add_subdirectory(lib)
add_subdirectory(app)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    add_subdirectory(tools)
endif()

install(DIRECTORY data DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_SYSTEM_PROCESSOR}")
file(GLOB PY_SCRIPTS "scripts/*.py")
install(PROGRAMS ${PY_SCRIPTS} DESTINATION "${CMAKE_INSTALL_PREFIX}"
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ GROUP_WRITE)
