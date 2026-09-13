add_executable(run_vector_test test_vector.cpp)
target_link_libraries(run_vector_test PRIVATE libgnc)


add_executable(run_quat_test test_quat.cpp)
target_link_libraries(run_quat_test PRIVATE libgnc)


add_executable(run_matrix_test test_matrix.cpp)
target_link_libraries(run_matrix_test PRIVATE libgnc)


add_executable(run_kalman_test test_kalman.cpp)
target_link_libraries(run_kalman_test PRIVATE libgnc)