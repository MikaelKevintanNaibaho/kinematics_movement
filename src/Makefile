# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c11 -g 
LDFLAGS = -lgsl -lgslcblas -lwiringPi -lm

# Source files
SRC = \
	main.c \
	pca9685.c \
	ik.c \
	move.c \
	dh.c \
	leg.c \
	bezier.c \
	interrupt.c \
	state_machine.c \
	trajectory.c \
	i2c_utils.c \
	i2c_interface.c \

LIB_EXCLUDE = \
			main.c \
			i2c_utils.c \

LIBRARY = $(BIN_DIR)/libpwmservo.a

LIB_OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(filter-out $(LIB_EXCLUDE),$(SRC)))

# Object files directory
OBJ_DIR = ../build/obj

# Executable directory
BIN_DIR = ../build/bin

# Object files
OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(SRC))

# Executable name
TARGET = $(BIN_DIR)/pwm_servo

# Formatting and Static Analysis tools
CLANG_FORMAT = clang-format
CPPCHECK = cppcheck

CPPCHECK_INCLUDES = ./

CPPCHECK_FLAGS = \
	--quiet --enable=all --error-exitcode=1 \
	--inline-suppr \
	--suppress=missingIncludeSystem \
	--suppress=unmatchedSuppression \
	--suppress=unusedFunction \
	$(addprefix -I,$(CPPCHECK_INCLUDES))

.PHONY: all test clean format check

all: $(TARGET)

# Build the main executable
$(TARGET): $(OBJ) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Compile Source files into object files
$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

test: $(LIBRARY)

#build static library
$(LIBRARY): $(LIB_OBJ) | $(BIN_DIR)
	ar rcs $@ $(LIB_OBJ)

# Compile Source files into object files
$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

format:
	$(CLANG_FORMAT) -i $(SRC) $(wildcard *.h)

cppcheck:
	$(CPPCHECK) $(CPPCHECK_FLAGS) $(SRC) $(wildcard *.h)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)/pwm_servo 
	rm -rf $(BIN_DIR) $(BIN_DIR)/libpwmservo.a
	rmdir --ignore-fail-on-non-empty $(BIN_DIR) $(OBJ_DIR) || true


