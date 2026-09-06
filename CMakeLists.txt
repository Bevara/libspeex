cmake_minimum_required(VERSION 3.7)
project(Filters)

include(filters.cmake)

add_definitions(-fpic)

SET(LIBSPEEX_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/dec_speex.c
)

SET(LIBSPEEX_LIBS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libspeex.a
)

SET(LIBSPEEX_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_filter(libspeex
        "${LIBSPEEX_SRC}"
        "${LIBSPEEX_LIBS}"
        []
        ""
        "${LIBSPEEX_INC}"
        ""
        "1")
