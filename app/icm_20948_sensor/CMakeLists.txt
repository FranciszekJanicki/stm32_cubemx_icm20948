add_library(icm_20948_sensor STATIC)

target_sources(icm_20948_sensor PRIVATE 
  "ICM_20948_C.c"
  "icm_sensor.c"
)

target_include_directories(icm_20948_sensor PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(icm_20948_sensor PRIVATE
    stm32cubemx
)

target_compile_options(icm_20948_sensor PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)
