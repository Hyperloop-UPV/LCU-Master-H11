#include "main.h"
#include "LCU_MASTER.hpp"

int main(void) {
    LCU_Master::init();

    while (1) {
        LCU_Master::update();
    }
}

void Error_Handler(void) {
    PANIC("HAL error handler triggered");
    while (1) {
    }
}
