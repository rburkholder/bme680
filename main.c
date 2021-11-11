#
# Read Bosch BME680 registers and compute values
# author:  raymond@burkholder.net
# started: 2021/11/10
#

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
*/

typedef unsigned char   u8;

int fd_i2c;
int id_adapter = 2;
u8 addr = 0x76; /* The I2C address */

void main() {

  char filename[20];
  u8 buf[0x20];

  snprintf( filename, 19, "/dev/i2c-%d", id_adapter );
  fd_i2c = open( filename, O_RDWR);
  if ( fd_i2c < 0 ) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    printf( "file error\n" );
  exit(1);
  }

  if ( 0 > ioctl( fd_i2c, I2C_SLAVE, addr) ) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    printf( "ioctl error\n" );
   exit(1);
  } 

  // chip id of BME680 in register 0xd0
  u8 reg = 0xd0;
  if ( 0 > write( fd_i2c, &reg, 1 ) ) {
    printf( "write failed\n" );
    exit( 1 );
  }

  u8 result;
  if ( 0 > read( fd_i2c, &result, 1 ) ) {
    printf( "read failed\n" );
    exit( 1 );
  }

  if ( 0x61 == result ) {
    printf( "found correct chip id\n" );
  }
  else {
    printf( "unexpected chip id = 0x%02x", result );
  }
   
//  if ( i2c_read( id_adapter, 0xD0, buf ) < 0 ) {
//  }
//  else {
//    printf( "result=0x%02x", buf[0] );
//  }

  printf( "done\n" );
}

