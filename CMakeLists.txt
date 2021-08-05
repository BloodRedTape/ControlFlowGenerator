cmake_minimum_required(VERSION 3.10)

project(FlowControlGenerator)


add_subdirectory(thirdparty/tinyxml2)
add_subdirectory(thirdparty/stb)

set(FCG_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sources)
set(FCG_THD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

set(FCG_SRC
	${FCG_SRC_DIR}/main.cpp
)

add_executable(FlowControlGenerator ${FCG_SRC})
target_link_libraries(FlowControlGenerator tinyxml2 stb_c_lexer clang)
target_include_directories(FlowControlGenerator PRIVATE ${FCG_SRC_DIR} ${FCG_THD_DIR} /usr/lib/llvm-12/include)
target_link_directories(FlowControlGenerator PRIVATE /usr/lib/llvm-12/lib)