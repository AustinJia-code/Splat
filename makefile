BUILD_DIR := build

CXX := g++
CXXFLAGS := $(shell pkg-config --cflags opencv4)
LDFLAGS := $(shell pkg-config --libs opencv4)

requirements:
	sudo apt install libopencv-dev

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

calibrate: $(BUILD_DIR)
	$(CXX) src/calibrate.cpp -o $(BUILD_DIR)/calibrate $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./calibrate

test_cam: $(BUILD_DIR)
	$(CXX) src/OV9715.cpp -o $(BUILD_DIR)/OV9715 $(CXXFLAGS) $(LDFLAGS)
	cd $(BUILD_DIR) && ./OV9715

clean:
	rm -rf $(BUILD_DIR)
	@for dir in data/out*; do \
		if [ -d "$$dir" ]; then \
			echo "Cleaning $$dir"; \
			rm -rf "$$dir"/*; \
		fi; \
	done

.PHONY: requirements calibrate test_cam clean