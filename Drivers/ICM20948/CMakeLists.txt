file(GLOB src_icm20948
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

set(inc_dir_icm20948
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(lib_icm20948 STATIC
    ${src_icm20948}
)

target_include_directories(lib_icm20948 PUBLIC
    ${inc_dir_icm20948}   # driver headers
)
# NO HAL dependency

