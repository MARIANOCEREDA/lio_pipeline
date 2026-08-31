.PHONY: build-cpp build-cpp-test clean

WS_ROOT := /home/dev/ws
LIO_PIPELINE_CPP_BUILD_DIR := $(WS_ROOT)/lio_pipeline/cpp/build
ROS_SETUP := /opt/ros/jazzy/setup.bash
JOBS := 4

build-cpp:
	cmake -S lio_pipeline/cpp -B lio_pipeline/cpp/build && \
	cmake --build lio_pipeline/cpp/build && \
	cmake --install lio_pipeline/cpp/build

build-cpp-test:
	cmake -S lio_pipeline/cpp -B lio_pipeline/cpp/build -DBUILD_TESTING=ON && \
	cmake --build lio_pipeline/cpp/build && \
	cmake --install lio_pipeline/cpp/build && \
	./lio_pipeline/cpp/build/lio_pipeline_cpp_test

build-ros2:
	@bash -c "source $(ROS_SETUP) && \
			cd $(WS_ROOT) && \
			colcon build \
			--symlink-install \
			--parallel-workers $(JOBS) \
			--packages-select lio_pipeline \
			--cmake-args -DLIO_PIPELINE_CPP_PREFIX=$(LIO_PIPELINE_CPP_BUILD_DIR) \
			--event-handlers console_cohesion+"

install-rosdeps:
	@bash -c "source $(ROS_SETUP) && \
			cd $(WS_ROOT) && \
			rosdep install --from-paths lio_pipeline --ignore-src -r -y"

build-all:
	@$(MAKE) build-cpp
	@$(MAKE) build-ros2

clean-cpp:
	rm -rf lio_pipeline/cpp/build

clean-ros2:
	rm -rf install
	rm -rf log
	rm -rf build

clean-all:
	@$(MAKE) clean-cpp
	@$(MAKE) clean-ros2