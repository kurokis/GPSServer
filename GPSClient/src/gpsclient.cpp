
#include "../../libraries/mytcp/tcpserver.h"
#include "../../libraries/mytcp/tcpclient.h"
#include "../../libraries/mygps/gps.h"

#include <unistd.h> // usleep
#include <thread>
#include <mutex>

std::mutex m; // for lock

void GPSThread();
void GPSHandler(const char * src, size_t len);

struct GPS_PAYLOAD gps_payload;

bool new_data_available = false;

int main(int argc, char const *argv[])
{
  std::thread gps_thread(&GPSThread);

  cout.precision(8);
  for(;;) {
    m.lock();
    if(new_data_available){
      cout << " x: " << gps_payload.position[0];
      cout << " y: " << gps_payload.position[1];
      cout << " z: " << gps_payload.position[2];
      cout << " vx: " << gps_payload.velocity[0];
      cout << " vy: " << gps_payload.velocity[1];
      cout << " vz: " << gps_payload.velocity[2] << endl;
      new_data_available = false;
    }
    m.unlock();
    usleep(1000);
  }
  gps_thread.join();
  return 0;
}

void GPSThread()
{
  tcp_client c;
  cout << "Connecting to server... ";
  c.start_connect("127.0.0.1" , 8000);
  cout << "connected." << endl;
  for(;;){
    c.recv_data(GPSHandler);
    usleep(1000);
  }
}

void GPSHandler(const char * src, size_t len)
{
  m.lock();
  char buf[CLIENT_BUF_SIZE];
  memcpy(buf, src, len);
  struct GPS_PAYLOAD * new_gps_payload = (struct GPS_PAYLOAD *)buf;


  gps_payload.status = new_gps_payload->status;
  for (int i=0;i<3;i++){
    gps_payload.position[i] = new_gps_payload->position[i];
    gps_payload.velocity[i] = new_gps_payload->velocity[i];
    gps_payload.r_var[i] = new_gps_payload->r_var[i];
    gps_payload.v_var[i] = new_gps_payload->v_var[i];
  }
  new_data_available = true;
  m.unlock();
}
