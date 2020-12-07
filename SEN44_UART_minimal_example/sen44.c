/*
 * Copyright (c) 2019, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sen44.h"
#include "sensirion_shdlc.h"
#include "sps_git_version.h"

#define SEN44_ADDR 0x00
#define SEN44_CMD_START_MEASUREMENT 0x00
#define SEN44_CMD_STOP_MEASUREMENT 0x01
#define SEN44_SUBCMD_MEASUREMENT_START_PMSGP { 0x02 }    
#define SEN44_CMD_READ_MEASUREMENT 0x03

#define SEN44_SUBCMD_READ_MEASUREMENT_DEFAULT { 0x07 }
#define SEN44_READ_MEASUREMENT_MODE_DEFAULT 0x00
#define SEN44_READ_MEASUREMENT_SIZE_DEFAULT 7

#define SEN44_CMD_READ_FAN_SPEED 0x40
#define SEN44_CMD_FAN_CLEAN_INTV 0x80
#define SEN44_CMD_FAN_CLEAN_INTV_LEN 5
#define SEN44_SUBCMD_READ_FAN_CLEAN_INTV 0x00
#define SEN44_CMD_START_FAN_CLEANING 0x56
#define SEN44_CMD_DEV_INFO 0xd0
#define SEN44_CMD_DEV_INFO_SUBCMD_GET_SERIAL { 0x03 }
#define SEN44_CMD_RESET 0xd3
#define SEN44_ERR_STATE(state) (SEN44_ERR_STATE_MASK | (state))

const char *sen44_get_driver_version() {
    return SPS_DRV_VERSION_STR;
}

int16_t sen44_probe() {
    char serial[SEN44_MAX_SERIAL_LEN];
    int16_t ret = sen44_get_serial(serial);

    return ret;
}

int16_t sen44_get_serial(char *serial) {
    struct sensirion_shdlc_rx_header header;
    uint8_t param_buf[] = SEN44_CMD_DEV_INFO_SUBCMD_GET_SERIAL;
    int16_t ret;

    ret = sensirion_shdlc_xcv(SEN44_ADDR, SEN44_CMD_DEV_INFO, sizeof(param_buf),
                              param_buf, SEN44_MAX_SERIAL_LEN, &header,
                              (uint8_t *)serial);
    if (ret < 0)
        return ret;

    if (header.state)
        return SEN44_ERR_STATE(header.state);

    return 0;
}

int16_t sen44_start_measurement() {
    struct sensirion_shdlc_rx_header header;
    uint8_t param_buf[] = SEN44_SUBCMD_MEASUREMENT_START_PMSGP;

    return sensirion_shdlc_xcv(SEN44_ADDR, SEN44_CMD_START_MEASUREMENT,
                               sizeof(param_buf), param_buf, 0, &header, NULL);
}

int16_t sen44_stop_measurement() {
    struct sensirion_shdlc_rx_header header;

    return sensirion_shdlc_xcv(SEN44_ADDR, SEN44_CMD_STOP_MEASUREMENT, 0, NULL,
                               0, &header, NULL);
}

int16_t sen44_read_measurement(struct sen44_measurement *measurement) {
    struct sensirion_shdlc_rx_header header;
    int i;
    int16_t ret;
    uint16_t idx;
    int8_t datasize;

    datasize = SEN44_READ_MEASUREMENT_SIZE_DEFAULT;                  

    union {
        uint16_t u16_value;
        int16_t s16_value;
    } val, data[datasize];
    
    uint8_t param_buf[] = SEN44_SUBCMD_READ_MEASUREMENT_DEFAULT;
                               
    ret = sensirion_shdlc_xcv(SEN44_ADDR, SEN44_CMD_READ_MEASUREMENT, sizeof(param_buf), param_buf, sizeof(data), &header, (uint8_t *)data);
    if (ret)
        return ret;

    if (header.data_len != sizeof(data))
        return SEN44_ERR_NOT_ENOUGH_DATA;

    idx = 0;
    val.u16_value = be16_to_cpu(data[idx].u16_value);
    measurement->mc_1p0 = val.u16_value;
    ++idx;
    val.u16_value = be16_to_cpu(data[idx].u16_value);
    measurement->mc_2p5 = val.u16_value;
    ++idx;
    val.u16_value = be16_to_cpu(data[idx].u16_value);
    measurement->mc_4p0 = val.u16_value;
    ++idx;
    val.u16_value = be16_to_cpu(data[idx].u16_value);
    measurement->mc_10p0 = val.u16_value;
    ++idx;
    val.s16_value = be16_to_cpu(data[idx].u16_value);
    measurement->voc_index = val.s16_value;
    ++idx;
    val.s16_value = be16_to_cpu(data[idx].u16_value);
    measurement->ambient_humidity = val.s16_value;
    ++idx;
    val.s16_value = be16_to_cpu(data[idx].u16_value);
    measurement->ambient_temperature = val.s16_value;
    ++idx;

    if (header.state)
        return SEN44_ERR_STATE(header.state);

    return 0;
}

int16_t sen44_get_fan_auto_cleaning_interval(uint32_t *interval_seconds) {
    struct sensirion_shdlc_rx_header header;
    uint8_t tx_data[] = {SEN44_SUBCMD_READ_FAN_CLEAN_INTV};
    int16_t ret;

    ret = sensirion_shdlc_xcv(
        SEN44_ADDR, SEN44_CMD_FAN_CLEAN_INTV, sizeof(tx_data), tx_data,
        sizeof(*interval_seconds), &header, (uint8_t *)interval_seconds);
    if (ret < 0)
        return ret;

    *interval_seconds = be32_to_cpu(*interval_seconds);

    if (header.state)
        return SEN44_ERR_STATE(header.state);

    return 0;
}

int16_t sen44_set_fan_auto_cleaning_interval(uint32_t interval_seconds) {
    struct sensirion_shdlc_rx_header header;
    uint8_t ix;
    uint8_t cleaning_command[SEN44_CMD_FAN_CLEAN_INTV_LEN];
    uint32_t value = be32_to_cpu(interval_seconds);

    cleaning_command[0] = SEN44_SUBCMD_READ_FAN_CLEAN_INTV;
    for (ix = 0; ix < sizeof(uint32_t); ix++)
        cleaning_command[ix + 1] = (uint8_t)(value >> (8 * ix));
    return sensirion_shdlc_xcv(
        SEN44_ADDR, SEN44_CMD_FAN_CLEAN_INTV, sizeof(cleaning_command),
        (const uint8_t *)cleaning_command, sizeof(interval_seconds), &header,
        (uint8_t *)&interval_seconds);
}

int16_t sen44_get_fan_auto_cleaning_interval_days(uint8_t *interval_days) {
    int16_t ret;
    uint32_t interval_seconds;

    ret = sen44_get_fan_auto_cleaning_interval(&interval_seconds);
    if (ret < 0)
        return ret;

    *interval_days = interval_seconds / (24 * 60 * 60);
    return ret;
}

int16_t sen44_set_fan_auto_cleaning_interval_days(uint8_t interval_days) {
    return sen44_set_fan_auto_cleaning_interval((uint32_t)interval_days * 24 *
                                                60 * 60);
}

int16_t sen44_start_manual_fan_cleaning() {
    struct sensirion_shdlc_rx_header header;

    return sensirion_shdlc_xcv(SEN44_ADDR, SEN44_CMD_START_FAN_CLEANING, 0,
                               NULL, 0, &header, NULL);
}

int16_t sen44_reset() {
    return sensirion_shdlc_tx(SEN44_ADDR, SEN44_CMD_RESET, 0, NULL);
}
