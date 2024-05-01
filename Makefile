# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c11 -g 
LDFLAGS = -lgsl -lgslcblas -lm

# Source files
SRC = \
	main.c \
	pwm_servo.c \
	ik.c \
	move.c \
	dh.c \
	leg.c \
	bezier.c \

# Object files directory
OBJ_DIR = build/obj

# Executable directory
BIN_DIR = build/bin

# Object files
OBJ = $(patsubst %.c,$(OBJ_DIR)/%.o,$(SRC))

# Executable name
TARGET = $(BIN_DIR)/pwm_servo

# Formatting and Static Analysis tools
CLANG_FORMAT = clang-format-12
CPPCHECK = cppcheck

CPPCHECK_INCLUDES = ./

CPPCHECK_FLAGS = \
	--quiet --enable=all --error-exitcode=1 \
	--inline-suppr \
	--suppress=missingIncludeSystem \
	--suppress=unmatchedSuppression \
	--suppress=unusedFunction \
	$(addprefix -I,$(CPPCHECK_INCLUDES))

.PHONY: all clean format check

all: $(TARGET)

$(TARGET): $(OBJ) | $(BIN_DIR)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET) $(LDFLAGS)

$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

format: .clang-format
	$(CLANG_FORMAT) -i $(SRC) $(wildcard *.h)

cppcheck:
	$(CPPCHECK) $(CPPCHECK_FLAGS)  $(SRC) $(wildcard *.h)

clean:
	rm -rf build

debug: $(TARGET)
	gdb $(TARGET)