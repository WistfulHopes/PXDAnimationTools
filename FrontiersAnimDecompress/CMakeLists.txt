cmake_minimum_required(VERSION 3.12)

project(FrontiersAnimDecompress CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/includes/acl
)

set(SOURCES
    FrontiersAnimDecompress.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    PREFIX ""
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:_DEBUG>
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
)

if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        _CRT_SECURE_NO_WARNINGS
    )
    target_compile_options(${PROJECT_NAME} PRIVATE /permissive-)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_SOURCE_DIR}
)
