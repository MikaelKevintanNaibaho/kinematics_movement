#include "hexapod.h"

int main(void) 
{
    PCA9685_init();
    bool debug = true;
    hexapod hexapod(debug);


    hexapod.step(1.0);

    return 0;


}