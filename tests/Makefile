# Compiler
CXX = gcc
CXXFLAGS = -Wall -Wextra -std=c11 -I../src -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lunity -pthread -lgsl -lgslcblas -lwiringPi -lm

# Source files for tests
TEST_SRC = $(wildcard *.c)
# Source files for the main program (not included in test build)
MAIN_SRC = $(filter-out %.c,$(wildcard ../src/*.c))

# Test directory
TEST_DIR = ../build/test/
# Object files directory for tests
TEST_OBJ_DIR = $(TEST_DIR)/obj

# Executable directory for tests
TEST_BIN_DIR = $(TEST_DIR)/bin/

# Object files for the test source files
TEST_OBJ = $(patsubst %.c,$(TEST_OBJ_DIR)/%.o,$(TEST_SRC))

# Object files from src directory (excluding main.o)
OBJ_DIR = ../build/obj
SRC_OBJ_FILES = $(filter-out $(OBJ_DIR)/main.o,$(wildcard $(OBJ_DIR)/*.o))

# Executable name
TARGET = $(TEST_BIN_DIR)/test

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(TEST_OBJ) $(SRC_OBJ_FILES) | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(TEST_OBJ) $(SRC_OBJ_FILES) $(LDFLAGS)

$(TEST_OBJ_DIR)/%.o: %.c | $(TEST_OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_OBJ_DIR):
	mkdir -p $(TEST_OBJ_DIR)

$(TEST_BIN_DIR):
	mkdir -p $(TEST_BIN_DIR)

test: $(TARGET)

clean:
	rm -rf $(TEST_DIR)

