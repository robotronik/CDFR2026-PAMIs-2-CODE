file(GLOB_RECURSE app_sources "main.cpp" "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
)
