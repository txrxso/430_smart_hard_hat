#ifndef GPS_EVENTS_H
#define GPS_EVENTS_H

#include <freertos/event_groups.h>

// event group for fresh GPS read requests
extern EventGroupHandle_t gpsEventGroup;
#define GPS_READ_REQUEST_BIT (1 << 0) 
#define GPS_READ_SUCCESS_BIT (1 << 1)
#endif