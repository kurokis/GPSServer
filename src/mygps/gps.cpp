#include "gps.h"

GPS::GPS()  : vendorID(0x067b),
	productID(0x2303),
	dev_name("/dev/ttyUSB0"),
	baudrate(4800)
{
	flags.new_gpgga_available = false;
}

void GPS::Open(){
	Serial::Open(dev_name, baudrate);
}

void GPS::ProcessIncomingBytes(){
	const int message_buffer_length = 256;
	static char message_buffer[message_buffer_length];
	static char * message_buffer_ptr = &message_buffer[0];
	uint8_t read_buffer[32];

	uint8_t bytes_remaining = Serial::Read(read_buffer,sizeof(read_buffer));
	if(bytes_remaining == 0){
		return;
	}

	for(int i = 0; i < bytes_remaining; i++){
		bool new_message_available = false;
		char b = (char) read_buffer[i];
		switch(b){
			case '$':
				message_buffer_ptr = &message_buffer[0]; // reset pointer
				*message_buffer_ptr++ = b; // store byte into message buffer
				break;
			case '\r':
				*message_buffer_ptr++ = '\0'; // insert NULL into message_buffer
				new_message_available = true;
				break;
			default:
				*message_buffer_ptr++ = b; // store byte into message buffer
		}

		if(new_message_available){
			char message_type[5]; // GPGGA, etc.
			strncpy(message_type,&message_buffer[1],5);
			if(!strcmp(message_type,"GPGGA")){
				ProcessGPGGA(message_buffer);
			}
			ProcessPayload();
		}
	}
}

void GPS::ProcessGPGGA(char* message){
	gpgga.message = string(message);
	const int gpgga_buffer_length = 256;
	char buf[gpgga_buffer_length];
	strncpy(buf,message,gpgga_buffer_length);

	// return if payload is empty
	if(strstr(buf,",,,")){

		return;
	}

	char* saveptr; // use strtok_r because it is thread-safe
	strtok_r(buf,",",&saveptr); // "$GPGGA"
	char* gps_time_ = strtok_r(NULL,",",&saveptr); //
	char* latitude_ = strtok_r(NULL,",",&saveptr); // latitude dddmm.mmmm
	char* ns_ = strtok_r(NULL,",",&saveptr); // 'N' or 'S'
	char* longitude_ = strtok_r(NULL,",",&saveptr); // longitude dddmm.mmmm
	char* ew_ = strtok_r(NULL,",",&saveptr); // 'E' or 'W'
	char* fix_quality_ = strtok_r(NULL,",",&saveptr); // 0: no fix, 1: GPS, 2: DGPS
	char* satellites_ = strtok_r(NULL,",",&saveptr); // number of satellites
	char* hdop_ = strtok_r(NULL,",",&saveptr); // HDOP
	char* height_above_sea_level_ = strtok_r(NULL,",",&saveptr); // height above sea level
	strtok_r(NULL,",",&saveptr); // 'M'
	char* height_above_geoid_ = strtok_r(NULL,",",&saveptr); // height above geoid
	strtok_r(NULL,",",&saveptr); // 'M'
	// strtok_r will skip next item in the message, because it will always be
	// empty and strtok_r can't handle consecutive delimiters
	strtok_r(NULL,"*",&saveptr); // ID
	char* checksum_ = strtok_r(NULL,",",&saveptr); // checksum

	// Validate message with checksum.
	// Begin from character after '$' and end at character before '*'.
	int checksum_received = 0, checksum_computed = 0;
	if(checksum_[0]>='A'){
		checksum_received += 16*(int)(checksum_[0]-'A'+10);
	}else{
		checksum_received += 16*(int)(checksum_[0]-'0');
	}
	if(checksum_[1]>='A'){
		checksum_received += (int)(checksum_[1]-'A'+10);
	}else{
		checksum_received += (int)(checksum_[1]-'0');
	}
	char* ptr = &message[1]; // character after '$'
	while(*ptr != '*'){
		checksum_computed ^= (uint8_t)*ptr++;
	}
	if(checksum_received != checksum_computed){
		return;
	}
	gpgga.checksum = checksum_computed;

	// GPS time
	float time = atof(gps_time_);
	float h = floor(time/10000);
	float m = floor((time-h*10000)/100);
 	float s = time-h*10000-m*100;
	gpgga.gps_time = h*3600+m*60+s;

	// latitude and longitude
	float latitude = atof(latitude_);
	latitude = floor(latitude/100) + (latitude-100*floor(latitude/100))/60;
	if(ns_[0]=='S'){
		latitude *= -1;
	}
	gpgga.latitude = latitude;
	float longitude = atof(longitude_);
	longitude = floor(longitude/100) + (longitude-100*floor(longitude/100))/60;
	if(ew_[0]=='W'){
		longitude *= -1;
	}
	gpgga.longitude = longitude;

	// miscellaneous
	gpgga.fix_quality = atoi(fix_quality_);
	gpgga.satellites = atoi(satellites_);
	gpgga.hdop = atof(hdop_);
	gpgga.height_above_sea_level = atof(height_above_sea_level_);
	gpgga.height_above_geoid = atof(height_above_geoid_);

	flags.new_gpgga_available = true;
}

void GPS::ProcessPayload(){
	if(flags.new_gpgga_available){
		payload.position[0] = gpgga.longitude;
		payload.position[1] = gpgga.latitude;
		payload.position[2] = -gpgga.height_above_sea_level;
	}
	payload.velocity[0] = 0;
	payload.velocity[1] = 0;
	payload.velocity[2] = 0;
	payload.r_var[0] = 1.0;
	payload.r_var[1] = 1.0;
	payload.r_var[2] = 1.0;
	payload.v_var[0] = 0.1;
	payload.v_var[1] = 0.1;
	payload.v_var[2] = 0.1;
}

bool GPS::NewDataAvailable(){
	if(flags.new_gpgga_available){
		flags.new_gpgga_available = false;
		return true;
	}else{
		return false;
	}
}

void GPS::ShowData(){
	cout << "message: " << gpgga.message << endl;
	cout << "gpstime: " << gpgga.gps_time << endl;
	cout << "longitude: " << gpgga.longitude << endl;
	cout << "latitude: " << gpgga.latitude << endl;
	cout << "fix quality: " << gpgga.fix_quality << endl;
	cout << "satellites: " << gpgga.satellites << endl;
	cout << "hdop: " << gpgga.hdop << endl;
	cout << "height above sea level: " << gpgga.height_above_sea_level << endl;
	cout << "height above geoid: " << gpgga.height_above_geoid << endl;
	cout << "checksum: " << gpgga.checksum << endl;
}

const char* GPS::Payload(){
	return (const char *)&payload;
}

void GPS::Log(){
	cout << payload.position[0] << endl;
	fout << payload.position[2] << "," << payload.position[1] << "," << payload.position[2] << ",";
	fout << payload.velocity[0] << "," << payload.velocity[1] << "," << payload.velocity[2] << ",";
	fout << payload.r_var[0] << "," << payload.r_var[1] << "," << payload.r_var[2] << ",";
	fout << payload.v_var[0] << "," << payload.v_var[1] << "," << payload.v_var[2] << ",";
	fout << payload.status << endl;
}
