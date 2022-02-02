//
// Read Bosch BME680 registers and compute values
// author:  raymond@burkholder.net
// started: 2021/11/10
//

#include <fcntl.h>
#include <unistd.h>

#include <stdint.h>

#include <sys/types.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#ifndef BME680_H
#define BME680_H

/*
  sample code found at:
  https://elinux.org/Interfacing_with_I2C_Devices
  https://www.kernel.org/doc/html/v5.4/i2c/dev-interface.html
  https://gist.github.com/JamesDunne/9b7fbedb74c22ccc833059623f47beb7

  datasheet:
  https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf
*/

typedef unsigned char   u8;

// bme680 defines, from rev 1.6 of data sheet

// page 14 - control register in section 5.3.1.3
// sleep mode
#define op_mode_sleep  0x00
// triggers single measurement
#define op_mode_forced 0x01

// page 29
#define ctrl_op_mode           0x74

// page 30
#define ctrl_chip_id           0xd0

// page 15 - suggested oversampling - use one write command - 0x00 means do not perform
// first osrs_h
#define oversample_humidity    0x01
// second osrs_t - section 5.3.2.2
#define oversample_temperature 0x02
// third osrs_p
#define oversample_presssure   0x10

// ignore gas for now - but outline is on page 15

// page 16 has flow chart for all readings

// page 17 - section 3.3.1 - 
// temperature calculation formula

// page 18 - registers for value temperature retrieval:
#define calibration_parm_t1_msb 0xea
#define calibration_parm_t1_lsb 0xe9
#define calibration_parm_t2_msb 0x8b
#define calibration_parm_t2_lsb 0x8a
#define calibration_parm_t3     0x8c
#define temp_adc_hi             0x22
#define temp_adc_mid            0x23
#define temp_adc_lo             0x24
// page 30 section 5.3.2.2 - shared with pressure
#define temperature_control     0x74
#define temperature_os_skip     0x00
#define temperature_os_x1       0b00100000
#define temperature_os_x2       0b01000000
#define temperature_os_x4       0b01100000
#define temperature_os_x8       0b10000000
#define temperature_os_x16      0b10100000

// page 19 - registers for value pressure retrieval:
#define calibration_parm_p1_msb 0x8f
#define calibration_parm_p1_lsb 0x8e
#define calibration_parm_p2_msb 0x91
#define calibration_parm_p2_lsb 0x90
#define calibration_parm_p3     0x92
#define calibration_parm_p4_msb 0x95
#define calibration_parm_p4_lsb 0x94
#define calibration_parm_p5_msb 0x97
#define calibration_parm_p5_lsb 0x96
#define calibration_parm_p6     0x99
#define calibration_parm_p7     0x98
#define calibration_parm_p8_msb 0x9d
#define calibration_parm_p8_lsb 0x9c
#define calibration_parm_p9_msb 0x9f
#define calibration_parm_p9_lsb 0x9e
#define calibration_parm_pa     0xa0
#define pressure_adc_msb        0x1f
#define pressure_adc_mid        0x20
#define pressure_adc_lsb        0x21
// page 31 section 5.3.2.3 - shared with temperature
#define pressure_control        0x74
#define pressure_os_skip        0x00
#define pressure_os_x1          0b00000100
#define pressure_os_x2          0b00001000
#define pressure_os_x4          0b00001100
#define pressure_os_x8          0b00010000
#define pressure_os_x16         0b00010100

// page 20 - registers for value humidity retrieval:
#define calibration_parm_h1_msb 0xe3
#define calibration_parm_h1_lsb 0xe2
#define calibration_parm_h2_msb 0xe1
#define calibration_parm_h2_lsb 0xe2
#define calibration_parm_h3     0xe4
#define calibration_parm_h4     0xe5
#define calibration_parm_h5     0xe6
#define calibration_parm_h6     0xe7
#define calibration_parm_h7     0xe8
#define humidity_adc_msb        0x25
#define humidity_adc_lsb        0x26
// page 30 section 5.3.2.1
#define humidity_control        0x72
#define humidity_os_skip        0x00
#define humidity_os_x1          0b00000001
#define humidity_os_x2          0b00000010
#define humidity_os_x4          0b00000011
#define humidity_os_x8          0b00000100
#define humidity_os_x16         0b00000101

// page 31 section 5.3.2.4 - IIR filter control
//    temperature & pressure
#define iir_filter_control      0x75
#define iir_filter_coef_0       0b00000000
#define iir_filter_coef_1       0b00000100
#define iir_filter_coef_3       0b00001000
#define iir_filter_coef_7       0b00001100
#define iir_filter_coef_15      0b00010000
#define iir_filter_coef_31      0b00010100
#define iir_filter_coef_63      0b00011000
#define iir_filter_coef_127     0b00011100

// page 22 - registers for value heater:
#define heater_parm_g1          0xed
#define heater_parm_g2_msb      0xec
#define heater_parm_g2_lsb      0xeb
#define heater_parm_g3          0xee
#define heater_range            0x02
#define heater_value            0x00

// page 23 - registers for value gas:
#define gas_adc_msb             0x2a
#define gas_adc_lsb             0x2b
#define gas_range               0x2b
#define gas_range_sw_error      0x04

// page 36 - status registers
#define measurement_status      0x1d
#define status_new_data         0b10000000
#define status_measuring_gas    0b01000000
#define status_measuring        0b00100000
#define status_measuring_gas_ix 0b00001111

// end bme680 definitions

int open_i2c_bus( int id_bus );

int open_i2c_device( int id_bus, u8 id_device );

int read_registers( int fd, u8 reg, u8* data, int length );

int read_register( int fd, u8 reg, u8* data );

int write_register( int fd, u8 reg, u8 data );

int read_uint16( int fd, u8 reg, uint16_t* data );

int read_int16( int fd, u8 reg, int16_t* data );

int check_chip_id( int fd_bme680 );

struct temperature {
  uint16_t par_t1;
  int16_t par_t2;
  int8_t par_t3;
  int32_t raw;
  int32_t fine; // used in pressure
  int32_t compensated; // _.xx celcius
};

int read_temperature_calibration( int fd_bme680, struct temperature* t );

void compensate_temperature( struct temperature* t );

struct pressure {
  uint16_t par_p1;
  uint16_t par_p2;
  uint8_t  par_p3;
  uint16_t par_p4;
  uint16_t par_p5;
  uint8_t  par_p6;
  uint8_t  par_p7;
  uint16_t par_p8;
  uint16_t par_p9;
  uint8_t  par_p10;
  int32_t  raw;
  int32_t compensated; // pascal
};

int read_pressure_calibration( int fd_bme680, struct pressure* p );

void tp( struct pressure* p, const struct temperature* t );

void compensate_pressure( struct pressure* p, const struct temperature* t );

struct humidity {
  int32_t par_h1;
  int32_t par_h2;
  int32_t par_h3;
  int32_t par_h4;
  int32_t par_h5;
  int32_t par_h6;
  int32_t par_h7;
  uint32_t raw;
  int32_t compensated;
};

int read_humidity_calibration( int fd_bme680, struct humidity* h );

void compensate_humidity( struct humidity* h, const struct temperature* t );

int measure_pth( int fd_bme680, int32_t* pressure, int32_t* temperature, int32_t* humidity );

#endif

