cmake_minimum_required(VERSION 2.8.3)


add_executable(dummyListener dummyListener.cpp)
target_link_libraries( dummyListener ${catkin_LIBRARIES})
add_dependencies(dummyListener dummyListener_generate_messages_cpp)
