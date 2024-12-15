#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define NTP_POOL_SERVER_NAME "time.cloudflare.com"
#define TIMEZONE "CT" // PT, MT, MT_AZ, CT, ET, UTC

#define MQTT_ID "<id>"
#define MQTT_USER "<user>"
#define MQTT_PASSWORD "<pass>"
#define MQTT_BROKER_HOST "<ip>"
#define MQTT_BROKER_PORT 1883
#define MQTT_STATE_TOPIC "<path>/state"
#define MQTT_COMMANDS_TOPIC "<path>/commands"
#define MQTT_WILL_TOPIC "<path>/available"
#define MQTT_WILL_MESSAGE "offline"
#define MQTT_BIRTH_MESSAGE "online"

#endif //CONFIGURATION_H
