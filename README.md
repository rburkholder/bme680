## BME680 -> MQTT

* sudo apt install libpaho-mqtt-dev
* sudo apt install paho.mqtt.c-examples
* sudo apt install cmake

* https://packages.debian.org/bullseye/amd64/paho.mqtt.c-examples/filelist

```
mkdir build
cd build
cmake ..
cmake --build . --config Debug
# cmake --build . --config Release   # alternate build flavour
```
