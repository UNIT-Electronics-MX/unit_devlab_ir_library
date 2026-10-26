/**
 * @file DevLab_IR.cpp
 * @brief DevLab IRM-V838M3 infrared receiver driver (NEC) over DDP/I2C.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include "DevLab_IR.h"

bool DevLab_IR::begin() {
    _verified = false;
    _bus.setClock(_clock);
    _busReady = _bus.begin();
    if (!_busReady) return false;
    return (_verified = _ddp.matchesExpectedDevice(_address, &_info));
}

bool DevLab_IR::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock) {
    _verified = false;
    _clock = clock;
    _bus.setClock(_clock);
    _busReady = _bus.begin(sdaPin, sclPin);
    if (!_busReady) return false;
    return (_verified = _ddp.matchesExpectedDevice(_address, &_info));
}

bool DevLab_IR::beginRecovered(uint8_t sdaPin, uint8_t sclPin, uint32_t timeoutUs, bool restart) {
    _verified = false;
    _bus.setClock(_clock);
    _busReady = _bus.beginRecovered(sdaPin, sclPin, timeoutUs, restart);
    if (!_busReady) return false;
    return (_verified = _ddp.matchesExpectedDevice(_address, &_info));
}

void DevLab_IR::printInfo(Print &out) const {
    DevLabDDP::printDeviceInfo(out, _address, _info, _ddp.expectedDeviceId());
}

bool DevLab_IR::readCarrier(bool &carrier) {
    if (!_verified) return false;
    uint8_t raw = 0U;
    if (!_ddp.readCommand(_address, CMD_READ_LEVEL, &raw, 1U, _kResponseDelayMs)) return false;
    carrier = (raw & 0x01U) != 0U;
    return true;
}

bool DevLab_IR::readStatus(uint8_t &status) {
    if (!_verified) return false;
    return _ddp.readCommand(_address, CMD_READ_STATUS, &status, 1U, _kResponseDelayMs);
}

bool DevLab_IR::readRawFrame(uint8_t frame[4]) {
    if (!_verified) return false;
    return _ddp.readCommand(_address, CMD_READ_FRAME, frame, 4U, _kResponseDelayMs);
}

bool DevLab_IR::poll(DevLab_IR_Frame &out) {
    out = DevLab_IR_Frame();

    uint8_t status = 0U;
    if (!readStatus(status)) return false;

    out.repeat = (status & STATUS_REPEAT) != 0U;
    out.timingError = (status & STATUS_ERROR) != 0U;

    if (status & STATUS_NEW_FRAME) {
        uint8_t raw[4];
        if (!readRawFrame(raw)) return false;
        out.newFrame = true;
        out.extended = (uint8_t)(raw[0] ^ raw[1]) != 0xFFU;
        out.address = out.extended ? ((uint16_t)raw[0] | ((uint16_t)raw[1] << 8)) : raw[0];
        out.command = raw[2];
        out.valid = (status & STATUS_CHECK_OK) != 0U &&
                    (uint8_t)(raw[2] ^ raw[3]) == 0xFFU;
    }
    return true;
}

bool DevLab_IR::readRawInfo(DevLab_IR_RawInfo &info) {
    info = DevLab_IR_RawInfo();
    if (!supportsRaw()) return false;
    uint8_t raw[4];
    if (!_ddp.readCommand(_address, CMD_RAW_INFO, raw, 4U, _kResponseDelayMs)) return false;
    if (raw[0] > RAW_MAX_EDGES || raw[1] > raw[0] || (raw[2] & 0xF8U) != 0U) return false;
    info.edges = raw[0];
    info.pending = raw[1];
    info.available = (raw[2] & 0x01U) != 0U;
    info.truncated = (raw[2] & 0x04U) != 0U;
    info.capture = raw[3];
    return true;
}

bool DevLab_IR::readRawEdge(DevLab_IR_RawEdge &edge) {
    edge = DevLab_IR_RawEdge();
    if (!supportsRaw()) return false;
    uint8_t raw[4];
    if (!_ddp.readCommand(_address, CMD_RAW_EDGE, raw, 4U, _kResponseDelayMs)) return false;
    if ((raw[2] & 0x80U) == 0U) return false;  /* no edge returned */
    edge.durationUs = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    edge.carrier = (raw[2] & 0x01U) != 0U;
    edge.pending = raw[3];
    return true;
}

bool DevLab_IR::readDiagnostic(DevLab_IR_Diagnostic &diag) {
    diag = DevLab_IR_Diagnostic();
    if (!supportsDiagnostics()) return false;
    uint8_t raw[4];
    if (!_ddp.readCommand(_address, CMD_DIAGNOSTIC, raw, 4U, _kResponseDelayMs)) return false;
    diag.edgeCount = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    diag.inputHigh = (raw[2] & 0x02U) != 0U;
    diag.config = raw[3];
    return true;
}
