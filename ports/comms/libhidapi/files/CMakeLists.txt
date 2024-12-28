cmake_minimum_required(VERSION 3.6.3...3.25 FATAL_ERROR)

add_library(hidapi_openbsd
    ${HIDAPI_PUBLIC_HEADERS}
    hid.c
)
target_link_libraries(hidapi_openbsd PUBLIC hidapi_include)

find_package(Threads REQUIRED)

target_link_libraries(hidapi_openbsd PRIVATE Threads::Threads)

set_target_properties(hidapi_openbsd
    PROPERTIES
        EXPORT_NAME "openbsd"
        OUTPUT_NAME "hidapi-openbsd"
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        PUBLIC_HEADER "${HIDAPI_PUBLIC_HEADERS}"
)

# compatibility with find_package()
add_library(hidapi::openbsd ALIAS hidapi_openbsd)
# compatibility with raw library link
add_library(hidapi-openbsd ALIAS hidapi_openbsd)

if(HIDAPI_INSTALL_TARGETS)
    install(TARGETS hidapi_openbsd EXPORT hidapi
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hidapi"
    )
endif()

hidapi_configure_pc("${PROJECT_ROOT}/pc/hidapi-openbsd.pc.in")