cmake_minimum_required(VERSION 3.23)
project(Pin_GUI_Tese)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED NO_MODULE)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)

add_executable(Pin_GUI_Tese main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen pinocchio::pinocchio)
