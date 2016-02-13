#include "Headers/GPSUPDReceiver.h"

#define GPS_UDP_PORT 50000

GPSUDPReceiver::GPSUDPReceiver() : UDPReceiver(GPS_UDP_PORT,143) {

}

GPSUDPReceiver::~GPSUDPReceiver() {
}
