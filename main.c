//
// Read Bosch BME680 registers and compute values
// author:  raymond@burkholder.net
// started: 2021/11/10
//

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/*
  providees sample code:
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
// page 30 section 5.3.2.2
#define temperature_control     0x74

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
// page 31 section 5.3.2.3
#define pressure_control        0x74

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

// page 31 section 5.3.2.4 - IIR filter control
//    temperature & pressure
#define iir_filter_control      0x75

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

// end bme680 definitions

int open_i2c_bus( int id_bus ) {

  char filename[20];
  int fd;

  snprintf( filename, 19, "/dev/i2c-%d", id_bus );
  fd = open( filename, O_RDWR);
  if ( 0 > fd ) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    printf( "file error (%d)\n", fd );
  }

  return fd;
}

int open_i2c_device( int id_bus, u8 id_device ) {

  int fd;

  fd = open_i2c_bus( id_bus );
  if ( 0 <= fd ) {
    if ( 0 > ioctl( fd, I2C_SLAVE, id_device ) ) {
      /* ERROR HANDLING; you can check errno to see what went wrong */
      printf( "ioctl error (%d)\n", fd );
    }
  }

  return fd;

}

int read_register( int fd, u8 reg, u8* data ) {

  int result;

  result = write( fd, &reg, 1 );
  if ( 0 > result ) {
    printf( "register write failed (%d)\n", result );
  }
  else {
    result = read( fd, data, 1 );
    if ( 0 > result ) {
      printf( "register read failed (%d)\n", result );
    }
  }

  return result;

}

void main() {

  int i2c_id_bus = 2; /* beagleboard bus number */
  u8  i2c_addr_bme680 = 0x76; /* seeed bme680 board */

  int fd_bme680; /* file descripter for device when opened */

  u8 buf[0x20];

  fd_bme680 = open_i2c_device( i2c_id_bus, i2c_addr_bme680 );
  if ( 0 <= fd_bme680 ) {

    u8 data;

    int result = read_register( fd_bme680, ctrl_chip_id, &data );
    if ( 0 <= result ) {
      if ( 0x61 == data ) {
        printf( "found correct chip id\n" );
      }
      else {
        printf( "unexpected chip id = 0x%02x", result );
      }
    }

  }

  printf( "done\n" );
}

  // this does not appear to work, the device does not seem to understand I2C_RDWR
  // https://gist.github.com/JamesDunne/9b7fbedb74c22ccc833059623f47beb7

  // resorted to simple read/write commands
  // write with the register desired, then
  // read for the desired data

