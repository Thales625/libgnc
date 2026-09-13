BUILD_DIR := build

$(BUILD_DIR)/Makefile: CMakeLists.txt
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

build: $(BUILD_DIR)/Makefile
	cmake --build $(BUILD_DIR)

live: build
	python live.py

static: build
	python static.py

clean:
	rm -rf $(BUILD_DIR)

.PHONY: build live static clean
