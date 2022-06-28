/*!
 * @file libmcp9808.cpp
 *
 * I2C driver for Microchip MCP9808 Temp Sensor
 *
 * This is a library for the Adafruit MCP9808 breakout:
 * http://www.adafruit.com/products/1782
 *
 * @section author Author
 * Sunip Mukherjee (UMass Lowell)
 *
 * @section license License
 * BSD (see LICENSE)
 *
 * @section HISTORY
 *
 *  v0.1a - Alpha One
 */

#include "mcp9808.h"

int mcp9808::numDev = 0;

mcp9808::mcp9808()
{
    numDev++;
    return;
}

mcp9808::mcp9808(uint8_t addr)
{
    _status = false;
    if (begin(addr))
    {
        _status = true;
        numDev++;
    }
    return;
}

bool mcp9808::begin(uint8_t addr)
{
    _addr = addr;
    _i2cbus = open(I2C_FILE, O_RDWR);
    if (_i2cbus < 0)
    {
        perror("MCP9808: Begin: Opening Bus:");
        return false;
    }

    if (ioctl(_i2cbus, I2C_SLAVE, addr) < 0)
    {
        perror("MCP9808: Begin: ioctl: ");
        return false;
    }

    return init();
}

bool mcp9808::init()
{
    if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
        return false;
    if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
        return false;

    write16(MCP9808_REG_CONFIG, 0x0);
    numDev++;
    _status = true;
    return true;
}

mcp9808::~mcp9808()
{
    close(_i2cbus);
    numDev--;
}

short mcp9808::readTemp()
{
    float temp = 0xffff;
    uint16_t t = read16(MCP9808_REG_AMBIENT_TEMP);
    if (t != 0xFFFF)
    {
        temp = t & 0x0FFF;
        temp /= 16.0;
        if (t & 0x1000)
            temp -= 256;
    }
    if (_status)
        return (short)temp * 100;
    else
        return -8000;
}

void mcp9808::shutdown_wake(bool sw)
{
    uint16_t conf_shutdown;
    uint16_t conf_register = read16(MCP9808_REG_CONFIG);
    if (sw == true)
    {
        conf_shutdown = conf_register | MCP9808_REG_CONFIG_SHUTDOWN;
        write16(MCP9808_REG_CONFIG, conf_shutdown);
    }
    if (sw == false)
    {
        conf_shutdown = conf_register & ~MCP9808_REG_CONFIG_SHUTDOWN;
        write16(MCP9808_REG_CONFIG, conf_shutdown);
    }
}

void mcp9808::shutdown() { shutdown_wake(true); }

void mcp9808::wakeup()
{
    shutdown_wake(false);
    usleep(250000);
}

uint8_t mcp9808::getResolution()
{
    return read8(MCP9808_REG_RESOLUTION);
}

void mcp9808::setResolution(uint8_t val)
{
    write8(MCP9808_REG_RESOLUTION, val & 0x03);
}

uint16_t mcp9808::MfgID()
{
    return read16(MCP9808_REG_MANUF_ID);
}

uint16_t mcp9808::DevID()
{
    return read16(MCP9808_REG_DEVICE_ID);
}

void mcp9808::write16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    write(_i2cbus, &reg, 1);
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
    write(_i2cbus, &buf, 2);
    return;
}

uint16_t mcp9808::read16(uint8_t reg)
{
    uint8_t buf[2] = {0xff};
    write(_i2cbus, &reg, 1);
    read(_i2cbus, &buf, 2);
    return (((uint16_t)buf[0]) << 8 | buf[1]);
}

void mcp9808::write8(uint8_t reg, uint8_t val)
{
    write(_i2cbus, &reg, 1);
    write(_i2cbus, &val, 1);
    return;
}

uint8_t mcp9808::read8(uint8_t reg)
{
    write(_i2cbus, &reg, 1);
    uint8_t buf = 0xFF;
    read(_i2cbus, &buf, 1);
    return buf;
}