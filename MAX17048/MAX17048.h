/* MAX17048 Driver Library
 * Copyright (c) 2013 Neil Thiessen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAX17048_H
#define MAX17048_H

#include "mbed.h"

/** MAX17048 class.
 *  Used for controlling a MAX17048 fuel gauge connected via I2C.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "MAX17048.h"
 *
 * MAX17048 gauge(p28, p27);
 *
 * int main()
 * {
 *     //Try to open the MAX17048
 *     if (gauge.open()) {
 *         printf("Device detected!\n");
 *
 *         //Load the default compensation value
 *         gauge.compensation(MAX17048::RCOMP0);
 *
 *         while (1) {
 *             //Print the current state of charge
 *             printf("SOC = %f%%\n", (float)gauge);
 *
 *             //Sleep for 0.5 seconds
 *             wait(0.5);
 *         }
 *     } else {
 *         error("Device not detected!\n");
 *     }
 * }
 * @endcode
 */
class MAX17048
{
public:
    /** The default compensation value for the MAX17048
     */
    static const char RCOMP0;

    /** Represents the different alert flags for the MAX17048
     */
    enum AlertFlags {
        ALERT_RI = (1 << 0),  /**< Reset indicator */
        ALERT_VH = (1 << 1),  /**< Voltage high alert */
        ALERT_VL = (1 << 2),  /**< Voltage low alert */
        ALERT_VR = (1 << 3),  /**< Voltage reset alert */
        ALERT_HD = (1 << 4),  /**< SOC low alert */
        ALERT_SC = (1 << 5)   /**< SOC change alert */
    };

    /** Create a MAX17048 object connected to the specified I2C pins
     *
     * @param sda The I2C data pin.
     * @param scl The I2C clock pin.
     * @param hz The I2C bus frequency (defaults to 400kHz).
     */
    MAX17048(PinName sda, PinName scl, int hz = 400000);

    /** Probe for the MAX17048 and indicate if it's present on the bus
     *
     * @returns
     *   'true' if the device exists on the bus,
     *   'false' if the device doesn't exist on the bus.
     */
    bool open();

    /** Command the MAX17048 to perform a power-on reset
     */
    void reset();

    /** Command the MAX17048 to perform a QuickStart
     */
    void quickStart();

    /** Determine whether sleep mode is enabled on the MAX17048
     *
     * @returns
     *   'true' if sleep mode is enabled,
     *   'false' if sleep mode is disabled.
     */
    bool sleepEnabled();

    /** Enable or disable sleep mode on the MAX17048
     *
     * @param enabled Whether or not sleep mode is enabled.
     */
    void sleepEnabled(bool enabled);

    /** Determine whether or not the MAX17048 is hibernating
     *
     * @returns
     *   'true' if hibernating,
     *   'false' if not hibernating.
     */
    bool hibernating();

    /** Get the current hibernate threshold of the MAX17048
     *
     * @returns The current hibernate threshold in \%/hr.
     */
    float hibernateThreshold();

    /** Set the hibernate threshold of the MAX17048
     *
     * @param threshold The new hibernate threshold in \%/hr.
     */
    void hibernateThreshold(float threshold);

    /** Get the current active threshold of the MAX17048
     *
     * @returns The current active threshold in volts.
     */
    float activeThreshold();

    /** Set the active threshold of the MAX17048
     *
     * @param threshold The new active threshold in volts.
     */
    void activeThreshold(float threshold);

    /** Get the production version of the MAX17048
     *
     * @returns The 16-bit production version.
     */
    unsigned short version();

    /** Get the current compensation value of the MAX17048
     *
     * @returns The current compensation value as an unsigned char (0 to 255).
     */
    char compensation();

    /** Set the compensation value of the MAX17048
     *
     * @param rcomp The new compensation value as an unsigned char (0 to 255).
     */
    void compensation(char rcomp);

    /** Set the compensation value of the MAX17048 from the current cell temperature
     *
     * @param temp The current cell temperature in °C.
     */
    void tempCompensation(float temp);

    /** Determine whether or not the MAX17048 is in sleep mode
     *
     * @returns
     *   'true' if in sleep mode,
     *   'false' if not in sleep mode.
     */
    bool sleeping();

    /** Enter or exit sleep mode on the MAX17048 (sleep mode must be enabled first)
     *
     * @param sleep Whether or not to sleep.
     */
    void sleep(bool sleep);

    /** Determine whether or not the SOC 1% change alert is enabled on the MAX17048
     *
     * @returns
     *   'true' if enabled,
     *   'false' if not enabled.
     */
    bool socChangeAlertEnabled();

    /** Enable or disable the SOC 1% change alert on the MAX17048
     *
     * @param enabled Whether or the SOC 1% change alert is enabled.
     */
    void socChangeAlertEnabled(bool enabled);

    /** Determine whether or not the MAX17048 is asserting the ALRT pin
     *
     * @returns
     *   'true' if alerting,
     *   'false' if not alerting.
     */
    bool alerting();

    /** Command the MAX17048 to de-assert the ALRT pin
     */
    void clearAlert();

    /** Get the current SOC empty alert threshold of the MAX17048
     *
     * @returns The current SOC empty alert threshold in %.
     */
    char emptyAlertThreshold();

    /** Set the SOC empty alert threshold of the MAX17048
     *
     * @param threshold The new SOC empty alert threshold in %.
     */
    void emptyAlertThreshold(char threshold);

    /** Get the current low voltage alert threshold of the MAX17048
     *
     * @returns The current low voltage alert threshold in volts.
     */
    float vAlertMinThreshold();

    /** Set the low voltage alert threshold of the MAX17048
     *
     * @param threshold The new low voltage alert threshold in volts.
     */
    void vAlertMinThreshold(float threshold);

    /** Get the current high voltage alert threshold of the MAX17048
     *
     * @returns The current high voltage alert threshold in volts.
     */
    float vAlertMaxThreshold();

    /** Set the high voltage alert threshold of the MAX17048
     *
     * @param threshold The new high voltage alert threshold in volts.
     */
    void vAlertMaxThreshold(float threshold);

    /** Get the current reset voltage threshold of the MAX17048
     *
     * @returns The current reset voltage threshold in volts.
     */
    float vResetThreshold();

    /** Set the reset voltage threshold of the MAX17048
     *
     * @param threshold The new reset voltage threshold in volts.
     */
    void vResetThreshold(float threshold);

    /** Determine whether or not the reset voltage comparator is enabled on the MAX17048
     *
     * @returns
     *   'true' if enabled,
     *   'false' if not enabled.
     */
    bool comparatorEnabled();

    /** Enable or disable the reset voltage comparator on the MAX17048
     *
     * @param enabled Whether or not the reset voltage comparator is enabled.
     */
    void comparatorEnabled(bool enabled);

    /** Get the factory programmed 8-bit ID of the MAX17048
     *
     * @returns The 8-bit ID.
     */
    char id();

    /** Determine whether or not the voltage reset alert is enabled on the MAX17048
     *
     * @returns
     *   'true' if enabled,
     *   'false' if not enabled.
     */
    bool vResetAlertEnabled();

    /** Enable or disable the voltage reset alert on the MAX17048
     *
     * @param enabled Whether or the voltage reset alert is enabled.
     */
    void vResetAlertEnabled(bool enabled);

    /** Get the current alert flags on the MAX17048
     *
     * @returns The current alert flags as AlertFlags enum values OR'd together.
     */
    char alertFlags();

    /** Clear the specified alert flags on the MAX17048
     *
     * @param flags The alert flags to clear as AlertFlags enum values OR'd together.
     */
    void clearAlertFlags(char flags);

    /** Get the current cell voltage measurement of the MAX17048
     *
     * @returns The cell voltage measurement as a float.
     */
    float vcell();

    /** Get the current state of charge measurement of the MAX17048 as a float
     *
     * @returns The state of charge measurement as a float.
     */
    float soc();

    /** Get the current state of charge measurement of the MAX17048 as an int
     *
     * @returns The state of charge measurement as an int.
     */
    int soc_int();

    /** Get the current C rate measurement of the MAX17048
     *
     * @returns The C rate measurement as a float.
     */
    float crate();

#ifdef MBED_OPERATORS
    /** A shorthand for soc()
     *
     * @returns The state of charge measurement as a float.
     */
    operator float();

    /** A shorthand for soc_int()
     *
     * @returns The state of charge measurement as an int.
     */
    operator int();
#endif

private:
    //I2C register addresses
    enum Register {
        REG_VCELL       = 0x02,
        REG_SOC         = 0x04,
        REG_MODE        = 0x06,
        REG_VERSION     = 0x08,
        REG_HIBRT       = 0x0A,
        REG_CONFIG      = 0x0C,
        REG_VALRT       = 0x14,
        REG_CRATE       = 0x16,
        REG_VRESET_ID   = 0x18,
        REG_STATUS      = 0x1A,
        REG_TABLE       = 0x40,
        REG_CMD         = 0xFE
    };

    //Member constants
    static const int m_ADDR;

    //Member variables
    I2C m_I2C;

    //Internal functions
    unsigned short read(char reg);
    void write(char reg, unsigned short data);
};

#endif
