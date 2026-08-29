.PHONY: build-cpp build-cpp-test clean

build-cpp:
	cmake -S lio_pipeline/cpp -B lio_pipeline/cpp/build && \
	cmake --build lio_pipeline/cpp/build

build-cpp-test:
	cmake -S lio_pipeline/cpp -B lio_pipeline/cpp/build -DBUILD_TESTING=ON && \
	cmake --build lio_pipeline/cpp/build --target imu_test && \
	./lio_pipeline/cpp/build/imu_test

clean:
	rm -rf lio_pipeline/cpp/build