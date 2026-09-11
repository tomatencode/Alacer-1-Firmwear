#include <Arduino.h>
#include "HC12.hpp"

HC12::HC12(int setPin, int rxPin, int txPin, int baudRate, std::function<void()> onSendCallback, std::function<void()> onReceiveCallback)
    : _setPin(setPin), _rxPin(rxPin), _txPin(txPin), _baudRate(baudRate), _onSendCallback(onSendCallback), _onReceiveCallback(onReceiveCallback), _serial(_rxPin, _txPin), _atState(ATState::IDLE) {
}

void HC12::begin() {
    _serial.begin(_baudRate);

    pinMode(_setPin, OUTPUT);
    digitalWrite(_setPin, HIGH);
}

bool HC12::send(std::span<const uint8_t> data) {
    if (atBusy()) {
        return false;
    }
    if (_onSendCallback && data.size() > 0) {
        _onSendCallback();
    }
    for (auto byte : data) {
        _serial.write(byte);
    }
    return true;
}

bool HC12::available() {
    if (atBusy()) {
        return false;
    }
    return _serial.available() > 0;
}

std::optional<uint8_t> HC12::read() {
    if (atBusy()) {
        return std::nullopt;
    }
    if (!available()) {
        return std::nullopt;
    }
    if (_onReceiveCallback) {
        _onReceiveCallback();
    }
    return _serial.read();
}

bool HC12::sendATCommand(const char* command, uint32_t timeout_ms) {
    if (_atState != ATState::IDLE) {
        return false;
    }

    _atCommand = command;
    _atResponse.clear();
    _atStartStepTime = millis();
    _atTimeout = timeout_ms;

    digitalWrite(_setPin, LOW);
    _atState = ATState::ENTERING_AT_MODE;
    return true;
}

void HC12::update() {
    switch (_atState) {
        case HC12::ATState::IDLE:
            break; // Do nothing
        case HC12::ATState::ENTERING_AT_MODE: {
            if (millis() - _atStartStepTime > 100) { // wait for the module to enter AT mode
                _atStartStepTime = millis();
                _atState = ATState::AWAITING_RESPONSE;
                while (_serial.available()) _serial.read();
                _serial.print(_atCommand.c_str());
                _serial.print("\r\n");
            }
            break;
        }
        case HC12::ATState::AWAITING_RESPONSE: {
            while (_serial.available()) {
                _atResponse += static_cast<char>(_serial.read());
            }

            const bool responseComplete =
                _atResponse.size() >= 2 &&
                _atResponse[_atResponse.size() - 2] == '\r' &&
                _atResponse.back() == '\n';
            const bool timedOut =
                millis() - _atStartStepTime > _atTimeout;

            if (responseComplete || timedOut) {
                _atStartStepTime = millis();
                digitalWrite(_setPin, HIGH);
                _atState = ATState::EXITING_AT_MODE;
            }
            break;
        }
        case HC12::ATState::EXITING_AT_MODE:{
            if (millis() - _atStartStepTime > 100) { // wait for the module to exit AT mode
                _atState = ATState::DONE;
            }
            break;
        }
        case HC12::ATState::DONE:
            break; // gets reset to IDLE on takeAtResponse
    }
}

bool HC12::atBusy() {
    return _atState != ATState::IDLE && _atState != ATState::DONE;
}

bool HC12::atDone() {
    return _atState == ATState::DONE;
}

std::optional<etl::string<128>> HC12::takeATResponse() {
    if (atDone()) {
        etl::string<128> response = _atResponse;
        _atResponse.clear();
        _atState = ATState::IDLE;
        return response;
    }
    return std::nullopt;
}