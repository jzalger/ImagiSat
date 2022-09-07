#include "types.h"

String csv_string(environment_state position, int time) {
    String str = String(time);
    str = str + "," + String(position.alititude) + "," + String(position.latitude) + "," + String(position.longitude) + "," + String(position.sats);
    return str;
}
