add_executable(test_recovery_bt
  test_recovery_bt.cpp
)

ament_target_dependencies(test_recovery_bt
  ${dependencies}
)

#nstall(TARGETS test_recovery_bt RUNTIME DESTINATION share/${PROJECT_NAME})
install(TARGETS test_recovery_bt RUNTIME DESTINATION lib/${PROJECT_NAME})
#install(FILES test_recovery_bt_launch.py DESTINATION share/${PROJECT_NAME})

