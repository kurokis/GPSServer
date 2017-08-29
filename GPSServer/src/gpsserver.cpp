#include "../../libraries/tcp/tcpserver.h"
#include "../../libraries/tcp/tcpclient.h"
#include "../../libraries/gps/gps.h"
#include "../../libraries/serial/serial.hpp"

#include <iostream>
#include <unistd.h> // usleep

#define TCP_PORT_GPS (8000)

using namespace std;

int main(int argc, char const *argv[])
{
    GPS gps;

    #ifndef GPS_DEBUG_MODE
    cout << "Waiting for client... ";
    tcp_server s;
    // accept connection
    s.start_listen(TCP_PORT_GPS);
    s.start_accept();
    cout << "connected." << endl;
    #else
    cout << "DEBUG MODE" << endl;
    #endif

    gps.Open();
    for(;;){
      static int no_data_counter = 0;
      gps.ProcessIncomingBytes();
      if (gps.NewDataAvailable()) {
        no_data_counter = 0;
        #ifndef GPS_DEBUG_MODE
        s.send_data((const char*)gps.Payload(), sizeof(*gps.Payload()));
        #endif
        gps.Log();
        gps.ShowData();
      }else{
        if(++no_data_counter==1000){
            // no new data for 5 seconds
            cout << "NO NEW DATA " << no_data_counter  << "" << endl;;
            no_data_counter = 0;
        }
      }
      usleep(1000);
    }

    return 0;
}
