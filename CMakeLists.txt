add_executable(DoublePendulumMPC main.cpp ActuationModelDoublePendulum.cpp ActuationModelDoublePendulum.h CostModelDoublePendulum.cpp CostModelDoublePendulum.h Controller.cpp Controller.h)
target_include_directories(DoublePendulumMPC PUBLIC ${EIGEN3_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS} ${crocoddyl_INCLUDE_DIRS})
target_link_libraries(DoublePendulumMPC PUBLIC ${pinocchio_LIBRARIES} ${crocoddyl_LIBRARIES} yaml-cpp)
target_link_libraries(DoublePendulumMPC LINK_PUBLIC odrive_cpp)