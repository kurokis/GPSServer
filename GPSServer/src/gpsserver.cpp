#include "../../libraries/tcp/tcpserver.h"
#include "../../libraries/tcp/tcpclient.h"
#include "../../libraries/gps/gps.h"
#include "../../libraries/serial/serial.hpp"

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
        if(++no_data_counter==2000){
            // no new data for two seconds
            cout << "$$$ NO NEW DATA $$$" <<  endl;
            gps.ShowData();
            cout << "$$$$$$$$$$$$$$$$$$$" <<  endl;
            no_data_counter = 0;
        }
      }
      usleep(1000);
    }

    return 0;
}
