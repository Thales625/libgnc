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
	python scenarios/quadcopter/live.py

quadcopter_static: quadcopter_build
	python scenarios/quadcopter/static.py

# ------- AIRPLANE -------

airplane_build:
	$(MAKE) build SCENARIO=airplane

airplane_live: airplane_build
	python scenarios/airplane/live.py

airplane_static: airplane_build
	python scenarios/airplane/static.py