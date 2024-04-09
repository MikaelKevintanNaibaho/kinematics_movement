#include "hexapod.h"

int main(void) {
    PCA9685_init();

    hexapod walker(true);

    walker.angle[0] = 45 * DEGTORAD;
    walker.angle[1] = 150 * DEGTORAD;
    walker.angle[2] = 130 * DEGTORAD;

    walker.stand();

    sleep(3);

    walker.step(0.1);

    walker.sit();

    return 0;
}