#include "hexapod.h"

int main() {
  // Create a hexapod object with debug mode enabled for informative messages
  hexapod robot(true);

  // Set the initial speed and turning values (adjust these values as needed)
  robot.speed = 0.5; // Positive for forward, negative for backward (0 to 1.0)
  robot.turning = 0.2; // Positive for right turn, negative for left turn (-1.0 to 1.0)

  // Main loop for robot operation
  float dt = 0.01; // Time difference between steps (in seconds)
  while (true) {
    // Call the step function to update robot movement
    robot.step(dt);

    // Add your robot control logic here
    // For example, you could read sensor data and adjust speed/turning based on it

  }

  return 0;
}
