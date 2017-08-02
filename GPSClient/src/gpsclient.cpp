
#include "../../libraries/mytcp/tcpserver.h"
#include "../../libraries/mytcp/tcpclient.h"

#include <unistd.h> // usleep
#include <thread>
#include <mutex>

std::mutex m; // for lock

void GPSThread();
void GPSHandler(const char * src, size_t len);

struct GPS_PAYLOAD {
  float position[3]; // [m]
  float velocity[3]; // [m/s]
  float r_var[3]; // [m^2]
  float v_var[3]; // [m^2/s^2]
  uint8_t status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed)) gps_payload;

bool new_data_available = false;

int main(int argc, char const *argv[])
{
  std::thread gps_thread(&GPSThread);

  for(;;) {
    if(new_data_available){
      cout << "x: " << gps_payload.position[0] << endl;
      new_data_available = false;
    }
    usleep(1000);
  }
  gps_thread.join();
  return 0;
}

void GPSThread()
{
  tcp_client c;
  c.start_connect("127.0.0.1" , 8000);
  for(;;){
    c.recv_data(GPSHandler);
    usleep(1000);
  }
}

void GPSHandler(const char * src, size_t len)
{
  char buf[CLIENT_BUF_SIZE];
  memcpy(buf, src, len);
  struct GPS_PAYLOAD * new_gps_payload = (struct GPS_PAYLOAD *)buf;

  m.lock();
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
