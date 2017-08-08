#include "../../libraries/tcp/tcpserver.h"
#include "../../libraries/tcp/tcpclient.h"
#include "../../libraries/gps/gps.h"

#include <unistd.h> // usleep
#include <thread>
#include <mutex>

std::mutex m; // for lock

void GPSThread();
void GPSHandler(const char * src, size_t len);

struct GPSPayload gps_payload;

bool new_data_available = false;

int main(int argc, char const *argv[])
{
  std::thread gps_thread(&GPSThread);

  cout.precision(8);
  for(;;) {
    m.lock();
    if(new_data_available){
      cout << " longitude: " << gps_payload.longitude;
      cout << " latitude: " << gps_payload.latitude;
      cout << " z: " << gps_payload.z;
      cout << " vx: " << gps_payload.velocity[0];
      cout << " vy: " << gps_payload.velocity[1];
      cout << " vz: " << gps_payload.velocity[2];
      cout << " gps_status: " << gps_payload.gps_status << endl;
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
  struct GPSPayload * new_gps_payload = (struct GPSPayload *)buf;

  gps_payload.gps_status = new_gps_payload->gps_status;
  gps_payload.longitude = new_gps_payload->longitude;
  gps_payload.latitude = new_gps_payload->latitude;
  gps_payload.z = new_gps_payload->z;
  for (int i=0;i<3;i++){
    gps_payload.velocity[i] = new_gps_payload->velocity[i];
  }
  new_data_available = true;
  m.unlock();
}
