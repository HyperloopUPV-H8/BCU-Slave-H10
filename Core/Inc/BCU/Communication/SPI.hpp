#pragma once

#include "ST-LIB.hpp"
#include "Shared/SPI.hpp"
#include "Shared/StateMachine.hpp"

namespace BCU::Communication {

class SPI {
   public:
    StateMachine::state_id master_general_state{
        Shared::State::SharedStateMachine::GeneralState::Connecting};
    StateMachine::state_id master_nested_state{
        Shared::State::SharedStateMachine::NestedState::Idle};

   private:
    uint8_t spi_id;

    SPIStackOrder *state_order;

   public:
    SPI();

    SPI(Pin &spi_ready_slave_pin);

    void start(StateMachine::state_id *slave_general_state,
               StateMachine::state_id *slave_nested_state);

    void update();
};

};  // namespace BCU::Communication