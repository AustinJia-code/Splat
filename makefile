BUILD_DIR := build

CXX := g++
CXXFLAGS := $(shell pkg-config --cflags opencv4)
LDFLAGS := $(shell pkg-config --libs opencv4)

cpp_requirements:
	sudo apt install libopencv-dev
	sudo apt install libpcl-dev

python_requirements:
	pip install open3d
	pip install opencv-python

test:
	make clean
	make calibrate
	make test_cam
	make vis

run:
	make clean
	make calibrate
	make test_cam
	make vis

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

calibrate: $(BUILD_DIR)
	$(CXX) src/calibrate.cpp -o $(BUILD_DIR)/calibrate $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./calibrate

test_cam: $(BUILD_DIR)
	$(CXX) src/test_cam.cpp -o $(BUILD_DIR)/test_cam $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./test_cam

sweep: $(BUILD_DIR)
	$(CXX) src/sweep.cpp -o $(BUILD_DIR)/sweep $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./sweep

vis:
	python src/pcl.py

clean:
	rm -rf $(BUILD_DIR)
	@for dir in data/out*; do \
		if [ -d "$$dir" ]; then \
			echo "Cleaning $$dir"; \
			rm -rf "$$dir"/*; \
		fi; \
	done

.PHONY: requirements calibrate test_cam clean