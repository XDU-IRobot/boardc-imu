
add_library(libs INTERFACE)

add_subdirectory(librm)

target_link_libraries(libs INTERFACE
        rm
        eternal
)