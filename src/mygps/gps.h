#ifndef GPS_H_
#define GPS_H_

#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <string>

#include "../myserial/serial.hpp"

using namespace std;

// filestream for logging
static ofstream fout("../output_data/gps_log.csv", ios::out);

class GPS : public Serial
{
	private:
		uint16_t vendorID;
		uint16_t productID;
		string dev_name;
		int baudrate;
		struct FLAGS{
			bool new_gpgga_available;
		}flags;
		struct GPGGA{
			string message;
			float gps_time;
			float longitude;
			float latitude;
			int fix_quality;
			int satellites;
			float hdop;
			float height_above_sea_level;
			float height_above_geoid;
			int checksum;
		}gpgga;
		struct PAYLOAD {
			float position[3]; // [m]
			float velocity[3]; // [m/s]
			float r_var[3]; // [m^2]
			float v_var[3]; // [m^2/s^2]
			uint8_t status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
		} __attribute__((packed)) payload;

		void ProcessGPGGA(char* message);
		void ProcessPayload();
	public:
		// TODO: Add functionality to set home and waypoints
		// TODO: Add device scan using vendorID and productID
		GPS();
		void Open();
		void ProcessIncomingBytes();
		bool NewDataAvailable();
		void ShowData();
		const char* Payload();
		void Log();
};

#endif // GPS_H_
