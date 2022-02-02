
* sudo apt install libpaho-mqtt-dev
* sudo apt install paho.mqtt.c-examples
* sudo apt install cmake

* https://packages.debian.org/bullseye/amd64/paho.mqtt.c-examples/filelist
* https://www.eclipse.org/paho/files/mqttdoc/MQTTClient/html/index.html

```
mkdir build
cd build
cmake ..
cmake --build . --config Debug
# cmake --build . --config Release   # alternate build flavour


2022/02/01 - revised instructions
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
