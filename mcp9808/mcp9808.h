/*!
 * @file libmcp9808.h
 *
 * I2C driver for Microchip MCP9808 Temp Sensor
 *
 * This is a library for the Adafruit MCP9808 breakout:
 * http://www.adafruit.com/products/1782
 *
 * Program written by Sunip Mukherjee @ UMass Lowell
 *
 * BSD License (See LICENSE)
 *
 */

#ifndef LIBMCP9808_H
#define LIBMCP9808_H
/* C Headers */
extern "C"
{
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
}
/* C Headers */

/* Device Configuration */
#define MCP9808_I2CADDR_DEFAULT 0x18 ///< I2C address
#define MCP9808_REG_CONFIG 0x01      ///< MCP9808 config register

#define MCP9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolution
/* Device Configuration */

#ifndef I2C_FILE
#define I2C_FILE "/dev/i2c-1" // standard i2c device address for Raspberry Pi
#endif                        /* I2C_FILE */

class mcp9808
{
public:
    mcp9808();
    mcp9808(uint8_t addr);
    ~mcp9808();
    bool begin(uint8_t addr);

    uint16_t MfgID();
    uint16_t DevID();

    short readTemp();
    uint8_t getResolution(void);
    void setResolution(uint8_t val);

    void shutdown();
    void wakeup();

    static int numDev;

private:
    bool _status;
    uint8_t _addr;
    int _i2cbus;
    bool init();
    void write16(uint8_t reg, uint16_t val);
    uint16_t read16(uint8_t reg);
    void write8(uint8_t reg, uint8_t val);
    uint8_t read8(uint8_t reg);
    void shutdown_wake(bool sw);
};

#endif /* LIBMCP9808_H */