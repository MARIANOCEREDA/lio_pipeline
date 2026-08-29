.PHONY: build-cpp

build-cpp:
	cmake -S lio_pipeline/cpp -B lio_pipeline/cpp/build && \
	cmake --build lio_pipeline/cpp/build
