#include "main.h"
#include "ST-LIB.hpp"

int main(void) { 
  Hard_fault_check();
  STLIB::start();

  using myBoard = ST_LIB::Board<>;
  myBoard::init();

  while (1) {
    STLIB::update();
  }
}
void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}

