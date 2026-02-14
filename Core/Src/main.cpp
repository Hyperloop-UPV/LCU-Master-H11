#include "ST-LIB.hpp"
#include "main.h"
#include "Pinout/Pinout.hpp"
#include "LCU_MASTER.hpp"

using namespace ST_LIB;

int main(void) {
    
    LCU_Master::init();

    while (1) {
        LCU_Master::update();
    }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
