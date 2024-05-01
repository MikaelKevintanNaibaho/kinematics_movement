#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdbool.h>
#include <signal.h>
#include <stddef.h>

#ifndef _sigset_t_defined_
#define _sigset_t_defined_

typedef long sigset_t;

#endif /* _sigset_t_defined_ */

struct sigaction
{
    void (*sa_handler)(int); // Pointer to the signal handler function
    sigset_t sa_mask; // Set of signals to block while in handler
    int sa_flags; // Additional flags for signal handling
};

// Function prototypes
void setup_interrupt_handler(void (*handler)(int));
bool should_stop(void);
void interrupt_handler(int signum);

#endif /* INTERRUPT_H */
