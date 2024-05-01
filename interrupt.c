#include "interrupt.h"

// Global variable to indicate whether the interrupt flag is set
volatile sig_atomic_t interrupt_flag = false;

// Signal handler function
void interrupt_handler(int signum)
{
    // Set the flag to indicate an interrupt has occurred
    interrupt_flag = true;
}

// Function to initialize signal handler
void setup_interrupt_handler(void (*handler)(int))
{
    struct sigaction sa;
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
}

// Function to check if an interrupt has occurred
bool should_stop(void)
{
    return interrupt_flag;
}
