#ifndef _TYPES_H
#define _TYPES_H
#include <WString.h>

struct environment_state {
    int time;
    float latitude;
    float longitude;
    float alititude;
    int sats = 0;
    float temperature = 0.0;
    float pressure = 0.0;
    float humidity = 0.0;
    float voc = 0.0;
    float p_alt = 0.0;
};

struct DeviceState {
    float voltage = 0;
    int charge_state = 0;
    int n_messages = 0;
    int alerts = 0;
    uint64_t last_message_time = 0;
    uint64_t last_gps_lock_time = 0;
    int errors = 0;
};

struct Forecast {
    int time;
    float latitude;
    float longitude;
    float altitude;
    float temperature;
    float humidity;
    float pressure;
    float pop;
    float cape;
    float cloud_cover;
    float sfc_wind_mag;
    char sfc_wind_dir;
    String condition;
};

enum AlertType {
    LIGHTNING_ALERT,
    SEVERE_WEATHER_ALERT
};

enum AlertSource {
    WB_ALERT,
    ON_DEVICE_ALERT,
    IRIDIUM_ALERT
};


struct Alert {
    AlertType type;
    AlertSource source;
    String message;
};


class Data {
    public:
        int time;
        environment_state position;
};

String csv_string(environment_state position, int time);
#endif // _TYPES_H
