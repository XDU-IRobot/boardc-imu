
add_library(libs INTERFACE)

add_subdirectory(librm)

add_library(eternal INTERFACE)
target_include_directories(eternal INTERFACE ${CMAKE_CURRENT_LIST_DIR}/eternal/include)

target_link_libraries(libs INTERFACE
        rm
        eternal
)