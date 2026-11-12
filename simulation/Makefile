SCENARIO ?= quadcopter

BUILD_DIR := build/$(SCENARIO)

.PHONY: configure build clean quadcopter_live quadcopter_static airplane_live airplane_static

configure:
	cmake -B $(BUILD_DIR) -DSCENARIO=$(SCENARIO)

build: configure
	cmake --build $(BUILD_DIR)

clean:
	rm -rf build

# ------- QUADCOPTER -------

quadcopter_build:
	$(MAKE) build SCENARIO=quadcopter

quadcopter_live: quadcopter_build
	python -m scenarios.quadcopter.live

quadcopter_static: quadcopter_build
	python -m scenarios.quadcopter.static

# ------- AIRPLANE -------

airplane_build:
	$(MAKE) build SCENARIO=airplane

airplane_live: airplane_build
	python -m scenarios.airplane.live

airplane_static: airplane_build
	python -m scenarios.airplane.static