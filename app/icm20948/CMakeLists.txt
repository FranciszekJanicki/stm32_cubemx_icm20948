add_library(icm20948 STATIC)

target_sources(icm20948 PRIVATE 
    "icm20948.cpp"
)

target_include_directories(icm20948 PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(icm20948 PRIVATE
    utility
    stm32cubemx
    ak09916
)

target_compile_options(icm20948 PUBLIC
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
