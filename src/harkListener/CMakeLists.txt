cmake_minimum_required(VERSION 2.8.3)


add_executable(harkListener harkListener.cpp)
target_link_libraries( harkListener ${catkin_LIBRARIES})
add_dependencies( harkListener harkListener_generate_messages_cpp)

add_executable(harkListener2k harkListener2k.cpp)
target_link_libraries(harkListener2k ${catkin_LIBRARIES})
add_dependencies(harkListener2k harkListener2k_generate_messages_cpp)
