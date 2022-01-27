#include "SensorLib.h"
#include <iostream>

bool ready = true;

void printPacket(RangeFinderPacket& packet){
	std::cout << "Angle: " << packet.angle << std::endl;
	std::cout << "Distance: " << packet.distance << std::endl;
	ready = true;
}

void printPackets(std::vector<RangeFinderPacket>& packets){
	for(unsigned int i = 0; i < packets.size(); i++){
		std::cout << "Packet " << i << ":" << std::endl;
		printPacket(packets[i]);
	}
}

int main(){
	// General gpio initialization
	if(gpioInitialise() < 0){
		std::cout << "GPIO failed to initialize!" << std::endl;
		return -1;
	}else{
		std::cout << "GPIO initialized!" << std::endl;
	}

	// Initialize the Sensor object
	// The name of the variable can be set by defining SENSOR_NAME before including SensorLib.h - by default its just 'sensor'
	// Yes I hate this solution too
	sensor = new Sensor();

	// Get distance from angle 50 degrees and print the result
	std::cout << "Sending Angle Command..." << std::endl;
	sensor->getAngle(50.0, printPacket);
	ready = false;

	// Do other stuff here while arduino responds
	while(!ready);

	// Get distance from angle -20.5 degrees and print the result
	std::cout << "Sending Angle Command..." << std::endl;
	sensor->getAngle(-20.5, printPacket);
	ready = false;

	// Do other stuff here while arduino responds
	while(!ready);


	// Scan the full range of the sensor and print the results
	std::cout << "Sending Scan Command..." << std::endl;
	ready = false;

	// Do other stuff here while arduino responds
	sensor->scan(printPackets);
	

	// Get heading and rssi and print it
	// Note this command is blocking - may want to run it asyncronously
	std::cout << "Getting Radio Data..." << std::endl;
	float heading;
	uint8_t rssi;
	sensor->getHeadingRSSI(heading, rssi);
	std::cout << "Heading: " << heading << ", RSSI: " << (int)rssi << std::endl;

	return 0;
}

