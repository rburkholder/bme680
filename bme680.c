//
// Read Bosch BME680 registers and compute values
// author:  raymond@burkholder.net
// started: 2021/11/10
//

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "bme680.h"

/*
  sample code found at:
  https://elinux.org/Interfacing_with_I2C_Devices
  https://www.kernel.org/doc/html/v5.4/i2c/dev-interface.html
  https://gist.github.com/JamesDunne/9b7fbedb74c22ccc833059623f47beb7

  datasheet:
  https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf
*/

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

int read_registers( int fd, u8 reg, u8* data, int length ) {

  int result;

  result = write( fd, &reg, 1 );
  if ( 0 > result ) {
    printf( "register select failed (%d)\n", result );
  }
  else {
    result = read( fd, data, length );
    if ( 0 > result ) {
      printf( "register read failed (%d)\n", result );
    }
  }

  return result;

}

int read_register( int fd, u8 reg, u8* data ) {

  int result;

  result = read_registers( fd, reg, data, 1 );

  return result;

}

int write_register( int fd, u8 reg, u8 data ) {

  u8 buf[ 2 ];

  buf[ 0 ] = reg;
  buf[ 1 ] = data;

  int result = write( fd, &buf, 2 );
  if ( 0 > result ) {
    printf( "register write failed (%d)\n", result );
  }

  return result;

}

int read_int16( int fd, u8 reg, int16_t* data ) {

  u8 buf[ 2 ];

  int result = read_registers( fd, reg, buf, 2 );

  uint16_t t = (buf[ 1 ] << 8) | buf[0];
  *data = t;

  return result;
}

int read_uint16( int fd, u8 reg, uint16_t* data ) {

  u8 buf[ 2 ];

  int result = read_registers( fd, reg, buf, 2 );

  *data = buf[ 1 ];
  *data = ( *data << 8 ) | buf[ 0 ];

  return result;
}

int check_chip_id( int fd_bme680 ) {

  u8 data;

  int result = read_register( fd_bme680, ctrl_chip_id, &data );
  if ( 0 <= result ) {
    if ( 0x61 == data ) {
      printf( "found correct chip id\n" );
    }
    else {
      result = 0;
      printf( "unexpected chip id = 0x%02x", result );
    }
  }
  return result;
}

int read_temperature_calibration( int fd_bme680, struct temperature* t ) {

  int result;

  result = read_uint16(    fd_bme680, calibration_parm_t1_lsb, &t->par_t1 );
  result = read_int16(     fd_bme680, calibration_parm_t2_lsb, &t->par_t2 );
  result = read_register(  fd_bme680, calibration_parm_t3,     &t->par_t3 );

  return result;
}

void compensate_temperature( struct temperature* t ) {

  // it is interesting to note that the lower 3 bits of the raw temperature are thrown away
  int32_t var1 = ( t->raw >> 3 ) - ( (int32_t)t->par_t1 << 1 );
  int32_t var2 = ( var1 * (int32_t)t->par_t2 ) >> 11;
  int32_t var3 = ( ( ( ( var1 >> 1 ) * ( var1 >> 1 ) ) >> 12 ) * ( (int32_t)t->par_t3 << 4 ) ) >> 14;
  t->fine = var2 + var3;
  t->compensated = ( ( t->fine * 5 ) + 128 ) >> 8;

}

int read_pressure_calibration( int fd_bme680, struct pressure* p ) {

  int result;

  result = read_uint16(   fd_bme680, calibration_parm_p1_lsb, &p->par_p1  );
  result = read_uint16(   fd_bme680, calibration_parm_p2_lsb, &p->par_p2  );
  result = read_register( fd_bme680, calibration_parm_p3,     &p->par_p3  );
  result = read_uint16(   fd_bme680, calibration_parm_p4_lsb, &p->par_p4  );
  result = read_uint16(   fd_bme680, calibration_parm_p5_lsb, &p->par_p5  );
  result = read_register( fd_bme680, calibration_parm_p6,     &p->par_p6  );
  result = read_register( fd_bme680, calibration_parm_p7,     &p->par_p7  );
  result = read_uint16(   fd_bme680, calibration_parm_p8_lsb, &p->par_p8  );
  result = read_uint16(   fd_bme680, calibration_parm_p9_lsb, &p->par_p9  );
  result = read_register( fd_bme680, calibration_parm_pa    , &p->par_p10 );

  return result;
}

void comp_pres_dbl( struct pressure* p, const struct temperature* t ) {

  double var1, var2, var3, compensated;

  var1 = ( (double) t->fine / 2.0 ) - 64000.0;
  var2 = var1 * var1 * ( (double)p->par_p6 / 131072.0 );
  var2 = var2 + ( var1 * (double)p->par_p5 * 2.0 );
  var2 = ( var2 / 4.0 ) + ( (double)p->par_p4 * 65536.0 );
  var1 = ( ( ( (double)p->par_p3 * var1 * var1 ) / 16384.0 ) + ( (double)p->par_p2 * var1 ) ) / 524288.0;
  var1 = ( 1.0 + ( var1 / 32768.0 ) ) * (double)p->par_p1;
  compensated = 1048576.0 - (double)p->raw;
  compensated = ( ( compensated - ( var2 / 4096.0 ) ) * 6250.0 ) / var1;
  var1 = ( (double)p->par_p9 * compensated * compensated ) / 2147483648.0;
  var2 = compensated * ( (double)p->par_p8 / 32768.0 );
  double div = compensated / 256.0;
  var3 = div * div * div * ( p->par_p10 / 131072.0 );
  compensated = compensated + ( var1 + var2 + var3 + ( (double) p->par_p7 * 128.0 ) ) / 16.0;

  printf( "pressure=%d(%f)\n", p->raw, compensated );

}

void compensate_pressure( struct pressure* p, const struct temperature* t ) {

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t pressure_comp;

    /* This value is used to check precedence to multiplication or division
     * in the pressure compensation equation to achieve least loss of precision and
     * avoiding overflows.
     * i.e Comparing value, pres_ovf_check = (1 << 31) >> 1
     *
     * from https://github.com/BoschSensortec/BME68x-Sensor-API/blob/master/bme68x.c
     */
    const int32_t pres_ovf_check = INT32_C(0x40000000);

    /*lint -save -e701 -e702 -e713 */
    var1 = (((int32_t)t->fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)p->par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)p->par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)p->par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)p->par_p3 << 5)) >> 3) +
           (((int32_t)p->par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)p->par_p1) >> 15;
    pressure_comp = 1048576 - p->raw;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    if (pressure_comp >= pres_ovf_check) {
        pressure_comp = ((pressure_comp / var1) << 1);
    }
    else {
        pressure_comp = ((pressure_comp << 1) / var1);
    }

    var1 = ((int32_t)p->par_p9 * (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)p->par_p8) >> 13;
    var3 =
        ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
         (int32_t)p->par_p10) >> 17;
    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 + ((int32_t)p->par_p7 << 7)) >> 4);

    p->compensated = pressure_comp;

}

int read_humidity_calibration( int fd_bme680, struct humidity* h ) {

  int result;

  uint8_t e1, e2, e3;

  result = read_register( fd_bme680, calibration_parm_h2_msb, &e1 );
  result = read_register( fd_bme680, calibration_parm_h1_lsb, &e2 );
  result = read_register( fd_bme680, calibration_parm_h1_msb, &e3 );
  result = read_register( fd_bme680, calibration_parm_h3, &h->par_h3 );
  result = read_register( fd_bme680, calibration_parm_h4, &h->par_h4 );
  result = read_register( fd_bme680, calibration_parm_h5, &h->par_h5 );
  result = read_register( fd_bme680, calibration_parm_h6, &h->par_h6 );
  result = read_register( fd_bme680, calibration_parm_h7, &h->par_h7 );

  h->par_h1 = ( e3 << 4 ) | ( e2 & 0x0f );
  h->par_h2 = ( e1 << 4 ) | ( e2 >> 4 );

  return result;

}

void compensate_humidity( struct humidity* h, const struct temperature* t ) {

  int32_t var1 = h->raw - ( h->par_h1 << 4 ) - ( ( ( t->compensated * h->par_h3 ) / 100 ) >> 1 );
  int32_t var2
    = ( h->par_h2 * ( ( ( t->compensated * h->par_h4 ) / 100 )
    + ( ( ( t->compensated * ( ( t->compensated * h->par_h5 )
    / 100 ) ) ) >> 6 ) / 100 + ( 1 << 14 ) ) )>> 10;
  int32_t var3 = var1 * var2;
  int32_t var4 = ( ( h->par_h6 << 7) + ( ( t->compensated * h->par_h7) / 100) ) >> 4;
  int32_t var5 = ( (var3 >> 14 ) * ( var3 >> 14 ) ) >> 10;
  int32_t var6 = ( var4 * var5 ) >> 1;
  h->compensated = ( ( ( var3 + var6 ) >> 10 ) * 1000 ) >> 12;

}

int measure_pth( int fd_bme680, int32_t* temperature, int32_t* pressure, int32_t* humidity ) {

  int result;
  u8 status;

  // wait for any pending measurements to complete
  do {
    result = read_register( fd_bme680, measurement_status, &status );
  } while ( 0 < ( ( status_measuring | status_measuring_gas ) & status ) );

  // initiate a humidity, pressure, temperature measurement
  result = write_register( fd_bme680, humidity_control, humidity_os_x2 );
  result = write_register( fd_bme680, ctrl_op_mode, temperature_os_x2 | pressure_os_x2 | op_mode_forced );

  // wait for any pending measurements to complete
  int loops = 0;
  do {
    result = read_register( fd_bme680, measurement_status, &status );
    //printf( "loop %d: status=0x%02x\n", loops, status );
    loops++;
  } while ( 0 < ( ( status_measuring | status_measuring_gas ) & status ) );
  //printf( "waited %d loops\n", loops );

  if ( 0 < ( status_new_data & status ) ) {

    u8 bufTemperature[ 3 ];
    result = read_registers( fd_bme680, temp_adc_hi, bufTemperature, 3 );
    *temperature =                           bufTemperature[ 0 ];
    *temperature = ( *temperature << 8 ) |   bufTemperature[ 1 ];
    *temperature = ( *temperature << 4 ) | ( bufTemperature[ 2 ] >> 4 );

    u8 bufPressure[ 3 ];
    result = read_registers( fd_bme680, pressure_adc_msb, bufPressure, 3 );
    *pressure =                          bufPressure[ 0 ];
    *pressure = ( *pressure << 8 ) |     bufPressure[ 1 ];
    *pressure = ( *pressure << 4 ) | ( ( bufPressure[ 2 ] >> 4 ) & 0x0f );

    u8 bufHumidity[ 2 ];
    result = read_registers( fd_bme680, humidity_adc_msb, bufHumidity, 2 );
    *humidity =                      bufHumidity[ 0 ];
    *humidity = ( *humidity << 8 ) | bufHumidity[ 1 ];

  }
  else {
    printf( "no data available (0x%02x)\n", status );
  }

  return result;

}


  // this does not appear to work, the device does not seem to understand I2C_RDWR
  // https://gist.github.com/JamesDunne/9b7fbedb74c22ccc833059623f47beb7

  // resorted to simple read/write commands
  // write with the register desired, then
  // read for the desired data

