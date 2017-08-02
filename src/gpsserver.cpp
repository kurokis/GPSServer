#include "mytcp/tcpserver.h"
#include "mytcp/tcpclient.h"
#include "mygps/gps.h"
#include "myserial/serial.hpp"

#include <iostream>
#include <unistd.h> // usleep
using namespace std;

int main(int argc, char const *argv[])
{
    GPS gps;

    #ifndef GPS_DEBUG_MODE
    tcp_server s;
    // accept connection
    s.start_listen(8000);
    s.start_accept();
    #endif

    gps.Open();
  	for(;;){
      gps.ProcessIncomingBytes();
  		if (gps.NewDataAvailable()) {

        #ifndef GPS_DEBUG_MODE
        s.send_data(gps.Payload(), sizeof(gps.Payload()));
  			//savetofile(gps_);
        #else
        gps.ShowData();
        gps.Log();
        #endif

  		}
      usleep(1000);
  	}

    return 0;
}
