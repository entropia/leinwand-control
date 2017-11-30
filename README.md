# A Sensor reading the values from an BME280 and publishing them to a MQTT-Server

## Geting started
We would recommend using [platformio](https://platformio.org)

### Credentials

Two files are needed:
* platformio/src/MQTT_credentials.hpp

 ```cpp
#ifndef MQTT_CREDENTIALS_HPP
#define MQTT_CREDENTIALS_HPP

# define MQTT_SERVER      "mqtt.club.entropia.de"
# define MQTT_SERVER_PORT 1883
# define MQTT_TOPIC       "/public/sensoren/hauptraum"
# define MQTT_USER        "eve"
# define MQTT_PASSWORD    "entropia"

#endif
```

* platformio/src/WiFi_credentials.hpp

```cpp
#ifndef WIFI_CREDENTIALS_HPP
#define WIFI_CREDENTIALS_HPP

# define SSID   "entropolis"
# define WIFIPW "Iliketr41nz!"

#endif
```

Replace the values with your own configuration!
