add_library(ak09916 STATIC)

target_sources(ak09916 PRIVATE 
    "ak09916.cpp"
)

target_include_directories(ak09916 PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}
)


target_link_libraries(ak09916 PRIVATE
    utility
    stm32cubemx
)

target_compile_options(ak09916 PUBLIC
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
