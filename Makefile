# Directories
SRC_DIR = src
TEST_DIR = tests

# Targets
.PHONY: all clean format check test

all: $(SRC_DIR)/pwm_servo $(TEST_DIR)/test

# Build the main executable
$(SRC_DIR)/pwm_servo:
	$(MAKE) -C $(SRC_DIR)

# Build the tests
$(TEST_DIR)/test:
	$(MAKE) -C $(TEST_DIR)

# Run tests
test: 
	$(MAKE) -C $(SRC_DIR) test
	$(MAKE) -C $(TEST_DIR) test

# Clean everything
clean:
	$(MAKE) -C $(SRC_DIR) clean
	$(MAKE) -C $(TEST_DIR) clean

format:
	$(MAKE) -C $(SRC_DIR) format
	$(MAKE) -C $(TEST_DIR) format

cppcheck:
	$(MAKE) -C $(SRC_DIR) cppcheck

