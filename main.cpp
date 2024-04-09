#include "hexapod.h"

int main(void) 
{
    PCA9685_init();
    bool debug = true;
    hexapod hexapod(debug);


    hexapod.step(0.1);

    return 0;


}