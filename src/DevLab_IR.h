/**
 * @file DevLab_IR.h
 * @brief DevLab IRM-V838M3 infrared receiver driver (NEC) over DDP/I2C.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#ifndef DEVLAB_IR_H
#define DEVLAB_IR_H

#pragma once

#include "DevLabDDP.h"
#include "DevLabDDPConsole.h"
#include "DevLab_I2C_Orchestrator.h"

/* Result of DevLab_IR::poll(). */
struct DevLab_IR_Frame
{
    bool newFrame = false;   /* a complete NEC frame was received */
    bool repeat = false;     /* repeat code (key held down) */
    bool valid = false;      /* ~command check passed (newFrame only) */
    bool timingError = false;/* a frame with out-of-tolerance timing was seen */
    uint16_t address = 0U;   /* NEC address: 8 bits, or 16 bits if extended (newFrame only) */
    uint8_t command = 0U;    /* NEC command byte (newFrame only) */
    bool extended = false;   /* 16-bit address, no ~address byte (newFrame only) */
};

/* Result of DevLab_IR::readRawInfo() (command 0x83, firmware 1.2+). */
struct DevLab_IR_RawInfo
{
    uint8_t edges = 0U;      /* edges stored in the capture */
    uint8_t pending = 0U;    /* edges not yet read with readRawEdge() */
    uint8_t capture = 0U;    /* capture sequence number */
    bool available = false;  /* a capture is ready to be read */
    bool truncated = false;  /* the capture filled the buffer and lost edges */
};

/* Result of DevLab_IR::readRawEdge() (command 0x84, firmware 1.2+). */
struct DevLab_IR_RawEdge
{
    uint16_t durationUs = 0U;/* width of the pulse/gap in microseconds */
    bool carrier = false;    /* true: it follows a carrier burst; false: idle */
    uint8_t pending = 0U;    /* edges still left after this one */
};

/* Result of DevLab_IR::readDiagnostic() (command 0x85, firmware 1.5+). */
struct DevLab_IR_Diagnostic
{
    uint16_t edgeCount = 0U; /* total interrupt edges seen since reset */
    bool inputHigh = false;  /* current level of the receiver input pin */
    uint8_t config = 0U;     /* firmware pin configuration byte */
};

class DevLab_IR
{
public:
    /* DDP Device ID of the IRM-V838M3 module (DDP_DEVICE_ID in firmware). */
    static constexpr uint16_t DEVICE_ID = 0x0111U;

    /* Factory I2C address (STARTUP_I2C_ADDRESS). */
    static constexpr uint8_t DEFAULT_ADDRESS = 0x31U;

    /* Bits returned by readStatus() (command 0x81). */
    static constexpr uint8_t STATUS_NEW_FRAME = 0x01U;
    static constexpr uint8_t STATUS_REPEAT = 0x02U;
    static constexpr uint8_t STATUS_CHECK_OK = 0x04U;
    static constexpr uint8_t STATUS_CARRIER = 0x08U;
    static constexpr uint8_t STATUS_ERROR = 0x10U;

    explicit DevLab_IR(TwoWire &wire = Wire, uint8_t address = DEFAULT_ADDRESS, uint32_t clock = 400000UL)
    : _bus(wire, clock), _ddp(_bus, DEVICE_ID), _address(address), _clock(clock) {}

    bool begin();
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock = 400000UL);
    bool beginRecovered(uint8_t sdaPin, uint8_t sclPin, uint32_t timeoutUs = 20000, bool restart = false);

    /* Instantaneous module output (command 0x80): `carrier` is true while the
     * receiver detects a 38 kHz burst (output low). */
    bool readCarrier(bool &carrier);

    /* Reads the status byte (command 0x81). Reading CLEARS the event bits
     * (NEW_FRAME, REPEAT, ERROR) in the module. */
    bool readStatus(uint8_t &status);

    /* Reads the raw NEC frame (command 0x82): address, ~address, command,
     * ~command. Clears NEW_FRAME in the module. */
    bool readRawFrame(uint8_t frame[4]);

    /* One-call polling helper: reads the status and, if a new frame arrived,
     * the frame itself. Returns false on an I2C error. */
    bool poll(DevLab_IR_Frame &out);

    /* Raw pulse capture, protocol independent. readRawInfo() reports whether a
     * capture is ready; readRawEdge() then returns its edges one by one until
     * `pending` is 0. Each returns false on an I2C error or an inconsistent
     * response. Check supportsRaw() first. */
    bool readRawInfo(DevLab_IR_RawInfo &info);
    bool readRawEdge(DevLab_IR_RawEdge &edge);

    /* Edge counter and pin state (command 0x85). Check supportsDiagnostics(). */
    bool readDiagnostic(DevLab_IR_Diagnostic &diag);

    /* Firmware capabilities, from the version read by begin(). */
    bool supportsRaw() const { return _verified && _fwAtLeast(1U, 2U); }
    bool supportsDiagnostics() const { return _verified && _fwAtLeast(1U, 5U); }

    bool busReady() const { return _busReady; }
    bool isConnected() const { return _verified; }
    uint8_t address() const { return _address; }
    const DevLabDDP::DeviceInfo &deviceInfo() const { return _info; }
    void printInfo(Print &out = Serial) const;

    DevLabDDP::Master &protocol() { return _ddp; }
    DevLab_I2C_Orchestrator &bus() { return _bus; }

private:
    static constexpr uint8_t CMD_READ_LEVEL = 0x80U;
    static constexpr uint8_t CMD_READ_STATUS = 0x81U;
    static constexpr uint8_t CMD_READ_FRAME = 0x82U;
    static constexpr uint8_t CMD_RAW_INFO = 0x83U;
    static constexpr uint8_t CMD_RAW_EDGE = 0x84U;
    static constexpr uint8_t CMD_DIAGNOSTIC = 0x85U;
    static constexpr uint8_t RAW_MAX_EDGES = 80U;
    static constexpr uint16_t _kResponseDelayMs = 2U;

    bool _fwAtLeast(uint8_t major, uint8_t minor) const {
        return _info.firmwareMajor > major ||
               (_info.firmwareMajor == major && _info.firmwareMinor >= minor);
    }

    DevLab_I2C_Orchestrator _bus;
    DevLabDDP::Master _ddp;
    uint8_t _address;
    uint32_t _clock;
    bool _busReady = false;
    bool _verified = false;
    DevLabDDP::DeviceInfo _info;
};

#endif
