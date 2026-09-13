BUILD_DIR = build_pc
TEST_DIR = tests

.PHONY: all build test clean clean-all test-matrix test-vector test-kalman

all: build

# configure CMake project if necessary
$(BUILD_DIR)/CMakeCache.txt:
	mkdir -p $(BUILD_DIR)
	cmake -S . -B $(BUILD_DIR)

# compile test executables
build: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) -j

# run all tests with CTest
test: build
	cd $(BUILD_DIR) && ctest --output-on-failure

# run the matrix test
test-matrix: build
	./$(BUILD_DIR)/$(TEST_DIR)/run_matrix_test

# run the vector test
test-vector: build
	./$(BUILD_DIR)/$(TEST_DIR)/run_vector_test

# run the quaternion test
test-quat: build
	./$(BUILD_DIR)/$(TEST_DIR)/run_quat_test

# run the kalman test
test-kalman: build
	./$(BUILD_DIR)/$(TEST_DIR)/run_kalman_test

# clean build artifacts without deleting the build directory
clean:
	@if [ -d "$(BUILD_DIR)" ]; then cmake --build $(BUILD_DIR) --target clean; fi

# clean all build artifacts and delete the build directory
clean-all:
	rm -rf $(BUILD_DIR)