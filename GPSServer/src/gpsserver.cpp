#include "../../libraries/mytcp/tcpserver.h"
#include "../../libraries/mytcp/tcpclient.h"
#include "../../libraries/mygps/gps.h"
#include "../../libraries/myserial/serial.hpp"

#include <iostream>
#include <unistd.h> // usleep
using namespace std;

int main(int argc, char const *argv[])
{
    GPS gps;

    #ifndef GPS_DEBUG_MODE
    cout << "Waiting for client... ";
    tcp_server s;
    // accept connection
    s.start_listen(8000);
    s.start_accept();
    cout << "connected." << endl;
    #else
    cout << "DEBUG MODE" << endl;
    #endif

    gps.Open();
    for(;;){
      gps.ProcessIncomingBytes();
      if (gps.NewDataAvailable()) {
        #ifndef GPS_DEBUG_MODE
        s.send_data(gps.Payload(), sizeof(gps.Payload()));
        #endif
        gps.Log();
        gps.ShowData();
      }
      usleep(1000);
    }

    return 0;
}
