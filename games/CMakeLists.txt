
add_executable(games main.cpp)

target_link_libraries(
    games
    ${OpenCV_LIBS} 
)
