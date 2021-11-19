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

#include "bme680.h"

void main() {

  int i2c_id_bus = 2; /* seeed beagleboard green bus number */
  u8  i2c_addr_bme680 = 0x76; /* seeed bme680 board */

  int fd_bme680; /* file descripter for device when opened */

  fd_bme680 = open_i2c_device( i2c_id_bus, i2c_addr_bme680 );
  if ( 0 <= fd_bme680 ) {

    int result;

    result = check_chip_id( fd_bme680 );

    if ( 0 < result ) {

      // make sure iir filtering is off
      result = write_register( fd_bme680, iir_filter_control, 0x00 );

      // temperature seems a bit low by a degree or two
      // pressure seems high - about 200 or 300 too high
      // humidity seems low - a bit low by 4 or 5?

      struct temperature t;
      result = read_temperature_calibration( fd_bme680, &t );

      struct pressure p;
      result = read_pressure_calibration( fd_bme680, &p );

      struct humidity h;
      result = read_humidity_calibration( fd_bme680, &h );

      while ( 1 ) {
        result = measure_pth( fd_bme680, &p.raw, &t.raw, &h.raw );
        compensate_temperature( &t );
        compensate_pressure( &p, &t );
        compensate_humidity( &h, &t );
        printf( 
          "p=%d(%0.2f) mbar, t=%d(%0.2f) degC, h=%d(%0.3f)%\n", 
          p.raw, ( (double)p.compensated / 100.0 ), 
          t.raw, ( (double)t.compensated / 100.0 ), 
          h.raw, ( (double)h.compensated / 1000.0 ) 
          );
        sleep( 3 );
      }
    }
  }

  close( fd_bme680 );
  printf( "done\n" );
}

