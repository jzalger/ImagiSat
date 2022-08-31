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

class Data {
    public:
        int time;
        environment_state position;
};

String csv_string(environment_state position, int time);
#endif // _TYPES_H
