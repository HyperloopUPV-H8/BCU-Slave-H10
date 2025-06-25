#ifndef SIM_ON
#include "main.h"

#include "lwip.h"
#endif

#include "BCU/Board.hpp"

int main(void) {
    BCU::Board board;

    while (1) {
        board.update();
    }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
