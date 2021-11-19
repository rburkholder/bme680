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

#include <time.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <MQTTClient.h>

#include "bme680.h"

#define ADDRESS     "tcp://rbtmq01.duchess.burkholder.net:1883"
#define CLIENTID    "bb01" // convert to command line
#define TOPIC       "/beagle/bme680"
#define QOS         1
#define TIMEOUT     10000L

void main() {

  int i2c_id_bus = 2; /* seeed beagleboard green bus number */
  u8  i2c_addr_bme680 = 0x76; /* seeed bme680 board */

  int fd_bme680; /* file descripter for device when opened */

  fd_bme680 = open_i2c_device( i2c_id_bus, i2c_addr_bme680 );

  MQTTClient client;
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  MQTTClient_message pubmsg = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token;
  int rc;

  if ((rc = MQTTClient_create(
    &client, ADDRESS, CLIENTID,
    MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS) {
     printf("Failed to create client, return code %d\n", rc);
     exit(EXIT_FAILURE);
  }

  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.username = "beagle";  // convert to command line
  conn_opts.password = "beagle";  // convert to command line
  if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
    printf("Failed to connect, return code %d\n", rc);
    exit(EXIT_FAILURE);
  }

  #define max_buf_size 500
  char szMessage[ max_buf_size ];

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

      time_t now;
      struct tm timeinfo;
      unsigned char szTimeInfo[ 50 ];
      const unsigned char szTopic[] = "/beagle/bme680";

      while ( 1 ) {

        // Get current time and publish it
        time( &now );
        localtime_r( &now, &timeinfo );
        sprintf( szTimeInfo, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo.tm_year + 1900,
          timeinfo.tm_mon + 1,
          timeinfo.tm_mday,
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec);

        result = measure_pth( fd_bme680, &p.raw, &t.raw, &h.raw );
        compensate_temperature( &t );
        compensate_pressure( &p, &t );
        compensate_humidity( &h, &t );

        double temperature = (double)t.compensated / 100.0;
        double pressure    = (double)p.compensated / 100.0;
        double humidity    = (double)h.compensated / 1000.0;

        printf( 
          "p=%d(%0.2f) mbar, t=%d(%0.2f) degC, h=%d(%0.3f)%\n", 
          p.raw, pressure,
          t.raw, temperature,
          h.raw, humidity
          );

        int sizeMessage;

        sizeMessage = snprintf(
          szMessage, max_buf_size,
          "{\"device\":\"%s\",\"values\":{\"ti\":\"%s\",\"t\":%0.2f,\"p\":%0.2f,\"h\":%0.3f}}",
          CLIENTID, szTimeInfo, temperature, pressure, humidity
          );

        pubmsg.payload = szMessage;
        pubmsg.payloadlen = sizeMessage;
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        if ( (rc = MQTTClient_publishMessage(client, szTopic, &pubmsg, &token)) != MQTTCLIENT_SUCCESS) {
          printf("Failed to publish message, return code %d\n", rc);
        }
        else {
          rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
          printf("Message %d: %s=%s delivered\n", token, szTopic, szMessage );
        }

        printf( "\n" );

        sleep( 5 );
      }
    }
  }

  close( fd_bme680 );

  if ((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS)
    printf("Failed to disconnect, return code %d\n", rc);
  MQTTClient_destroy(&client);

  printf( "done\n" );
}

