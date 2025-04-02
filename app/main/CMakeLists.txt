add_executable(app)

target_sources(app PRIVATE 
    "main.cpp"
)

target_include_directories(app PRIVATE 
    ${CMAKE_SOURCE_DIR}/app/icm20948
)

target_link_libraries(app PRIVATE
    stm32cubemx
    icm20948
    ak09916
    utility
)

target_compile_options(app PUBLIC
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
