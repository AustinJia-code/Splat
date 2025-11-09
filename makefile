BUILD_DIR := build

CXX := g++
CXXFLAGS := -std=c++17 `pkg-config --cflags opencv4`
LDFLAGS := `pkg-config --libs opencv4`

cpp_requirements:
	sudo apt install libopencv-dev
	sudo apt install libpcl-dev

python_requirements:
	pip install open3d
	pip install opencv-python

run:
	make clean
	make calibrate
	make process_scene
	make vis

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

calibrate: $(BUILD_DIR)
	$(CXX) src/calibrate.cpp -o $(BUILD_DIR)/calibrate $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./calibrate

process_scene: $(BUILD_DIR)
	$(CXX) src/process_scene.cpp -o $(BUILD_DIR)/process_scene $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./process_scene

record_scene: $(BUILD_DIR)
	$(CXX) src/record_scene.cpp -o $(BUILD_DIR)/record_scene $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./record_scene

vis_scan:
	python src/pcl_scan.py

vis_sweep:
	python src/pcl_sweep.py

clean:
	rm -rf $(BUILD_DIR)

.PHONY: requirements calibrate test_cam clean