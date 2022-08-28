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
};

class Data {
    public:
        int time;
        environment_state position;
};

String csv_string(environment_state position, int time);
#endif // _TYPES_H
