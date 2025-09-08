# Drivers/ICM20948/CMakeLists.txt

set(inc_dir_icm20948
    ${CMAKE_CURRENT_SOURCE_DIR}   # points to Drivers/ICM20948
)

add_library(lib_icm20948 INTERFACE)

# Expose the header folder so other targets can #include "icm20948.h"
target_include_directories(lib_icm20948 INTERFACE
    ${inc_dir_icm20948}
    ${MX_Include_Dirs}
)