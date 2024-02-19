/*
 * This file is part of lidar-contrib, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Armabot <https://www.armabot.com>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package frc.util.armabot.lidar.impl.vl6180x;

import frc.util.armabot.lidar.arcompat.Register;
import frc.util.armabot.lidar.util.Preconditions;

public enum Vl6180xReg implements Register {
    IDENTIFICATION__MODEL_ID(0x000),
    IDENTIFICATION__MODEL_REV_MAJOR(0x001),
    IDENTIFICATION__MODEL_REV_MINOR(0x002),
    IDENTIFICATION__MODULE_REV_MAJOR(0x003),
    IDENTIFICATION__MODULE_REV_MINOR(0x004),
    IDENTIFICATION__DATE_HI(0x006),
    IDENTIFICATION__DATE_LO(0x007),
    IDENTIFICATION__TIME(0x008), // 16-bit

    SYSTEM__MODE_GPIO0(0x010),
    SYSTEM__MODE_GPIO1(0x011),
    SYSTEM__HISTORY_CTRL(0x012),
    SYSTEM__INTERRUPT_CONFIG_GPIO(0x014),
    SYSTEM__INTERRUPT_CLEAR(0x015),
    SYSTEM__FRESH_OUT_OF_RESET(0x016),
    SYSTEM__GROUPED_PARAMETER_HOLD(0x017),

    SYSRANGE__START(0x018),
    SYSRANGE__THRESH_HIGH(0x019),
    SYSRANGE__THRESH_LOW(0x01A),
    SYSRANGE__INTERMEASUREMENT_PERIOD(0x01B),
    SYSRANGE__MAX_CONVERGENCE_TIME(0x01C),
    SYSRANGE__CROSSTALK_COMPENSATION_RATE(0x01E), // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT(0x021),
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE(0x022), // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET(0x024),
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT(0x025),
    SYSRANGE__RANGE_IGNORE_THRESHOLD(0x026), // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT(0x02C),
    SYSRANGE__RANGE_CHECK_ENABLES(0x02D),
    SYSRANGE__VHV_RECALIBRATE(0x02E),
    SYSRANGE__VHV_REPEAT_RATE(0x031),

    SYSALS__START(0x038),
    SYSALS__THRESH_HIGH(0x03A),
    SYSALS__THRESH_LOW(0x03C),
    SYSALS__INTERMEASUREMENT_PERIOD(0x03E),
    SYSALS__ANALOGUE_GAIN(0x03F),
    SYSALS__INTEGRATION_PERIOD(0x040),

    RESULT__RANGE_STATUS(0x04D),
    RESULT__ALS_STATUS(0x04E),
    RESULT__INTERRUPT_STATUS_GPIO(0x04F),
    RESULT__ALS_VAL(0x050), // 16-bit
    RESULT__HISTORY_BUFFER_0(0x052), // 16-bit
    RESULT__HISTORY_BUFFER_1(0x054), // 16-bit
    RESULT__HISTORY_BUFFER_2(0x056), // 16-bit
    RESULT__HISTORY_BUFFER_3(0x058), // 16-bit
    RESULT__HISTORY_BUFFER_4(0x05A), // 16-bit
    RESULT__HISTORY_BUFFER_5(0x05C), // 16-bit
    RESULT__HISTORY_BUFFER_6(0x05E), // 16-bit
    RESULT__HISTORY_BUFFER_7(0x060), // 16-bit
    RESULT__RANGE_VAL(0x062),
    RESULT__RANGE_RAW(0x064),
    RESULT__RANGE_RETURN_RATE(0x066), // 16-bit
    RESULT__RANGE_REFERENCE_RATE(0x068), // 16-bit
    RESULT__RANGE_RETURN_SIGNAL_COUNT(0x06C), // 32-bit
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT(0x070), // 32-bit
    RESULT__RANGE_RETURN_AMB_COUNT(0x074), // 32-bit
    RESULT__RANGE_REFERENCE_AMB_COUNT(0x078), // 32-bit
    RESULT__RANGE_RETURN_CONV_TIME(0x07C), // 32-bit
    RESULT__RANGE_REFERENCE_CONV_TIME(0x080), // 32-bit

    RANGE_SCALER(0x096), // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

    READOUT__AVERAGING_SAMPLE_PERIOD(0x10A),
    FIRMWARE__BOOTUP(0x119),
    FIRMWARE__RESULT_SCALER(0x120),
    I2C_SLAVE__DEVICE_ADDRESS(0x212),
    INTERLEAVED_MODE__ENABLE(0x2A3),
    ;

    private final short address;

    Vl6180xReg(int address) {
        Preconditions.checkArgument(0 <= address && address <= Short.MAX_VALUE, "address must be a positive short");
        this.address = (short) address;
    }

    public short address() {
        return address;
    }
}
