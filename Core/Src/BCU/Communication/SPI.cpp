#include "BCU/Communication/SPI.hpp"

namespace BCU::Communication {

SPI::SPI() : spi_id(::SPI::inscribe(::SPI::spi3)), state_order(nullptr) {}

SPI::SPI(Pin &spi_ready_slave_pin) : SPI() {
    ::SPI::assign_RS(spi_id, spi_ready_slave_pin);
}

static void state_callback() {}

void SPI::start(StateMachine::state_id *slave_general_state,
                StateMachine::state_id *slave_nested_state) {
    state_order = Shared::Communication::create_state_order(
        &master_general_state, &master_nested_state, slave_general_state,
        slave_nested_state);
    ::SPI::slave_listen_Orders(spi_id);
    state_order->set_callback(state_callback);
}

void SPI::update() { ::SPI::Order_update(); }

};  // namespace BCU::Communication