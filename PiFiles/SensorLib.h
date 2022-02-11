#ifndef SENSORLIB_H
#define SENSORLIB_H

#include <pigpio.h>
#include <iostream>
#include <vector>
#include <functional>
#include <cmath>
#include <bitset>
#include <functional>

#ifndef SENSOR_NAME
#define SENSOR_NAME sensor
#endif

#define SCAN_POINTS 10

#define INT_STATE_NONE 0
#define INT_STATE_GET_ANGLE 1
#define INT_STATE_SCAN 2

#define HANDSHAKE_REGISTER 0x01
#define SCAN_REGISTER 0x02
#define READ_SCAN_REGISTER 0x03
#define ANGLE_REGISTER 0x04
#define READ_ANGLE_REGISTER 0x05
#define HEADING_REGISTER 0x06
#define RSSI_REGISTER 0x07

#define ERROR_CONNECTION -1
#define ERROR_BUSY -2

#define INT_PIN 14

struct RangeFinderPacket{
	float angle;
	uint16_t distance;
}__attribute__((packed));

class Sensor{
public:
	Sensor();
	Sensor(uint8_t address, uint8_t bus);
	~Sensor();

	// Takes distance measurements at every point within the scan range
	// Arguments:
	// 	callbackFcn - 
	// 		Function which is called when the arduino responds with data.
	// 		callbackFcn should have no return value and should take a vector of RangeFinderPacket values as an argument
	// 		The vector will contain all the data returned from the arduino	
	//
	// Returns 0 if sucessful - othewise a negative error code
	int8_t scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn);

	
	// Gets distance measurement from a specified angle
	// Arguments:
	// 	angle -
	// 		float which specifies the angle to get a measurement from
	//
	// 	callbackFcn -
	// 		Function which is called when the arduino responds with data
	// 		callbackFcn should have no return value and should take a RangeFinderPacket value as an argument
	// 		The RangeFinderPacket argument contains the data returned from the arduino
	//
	// Returns 0 if sucessful - otherwise a negative error code
	int8_t getAngle(float angle, std::function<void(RangeFinderPacket&)> callbackFcn);

	// Gets the angle repored by the beacon with the lowest RSSI value
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments: none
	//
	// Returns the angle as a float
	float getHeading();

	// Gets the lowest RSSI value from the beacon
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments: none
	//
	// Returns the RSSI value as an unsigned 8 bit integer
	uint8_t getRSSI();

	// Gets the angle and RSSI value from the lowest reported rssi from the beacon
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments:
	// 	heading -
	// 		float to contain the heading value
	// 	rssi -
	// 		byte to contain the rssi value
	//
	// No return value - all values are set via pass by reference
	void getHeadingRSSI(float& heading, uint8_t& rssi);

	// Internal function
	// Handles interrupt response from arduino
	void intHandler(int gpio, int level, uint32_t tick);

private:
	uint8_t _addr;
	int _fd;
	uint8_t _bus;
	
	uint8_t _handshakeRegister = HANDSHAKE_REGISTER;
	uint8_t _scanRegister = SCAN_REGISTER;
	uint8_t _readScanRegister = READ_SCAN_REGISTER;
	uint8_t _angleRegister = ANGLE_REGISTER;
	uint8_t _readAngleRegister = READ_ANGLE_REGISTER;
	uint8_t _headingRegister = HEADING_REGISTER;
	uint8_t _rssiRegister = RSSI_REGISTER;

	uint8_t _interruptState = 0;

	std::function<void(std::vector<RangeFinderPacket>&)> _scanCallback;

	std::function<void(RangeFinderPacket&)> _angleCallback;
	void constructorUni();

};

// This part declares the sensor object to be used and is the interrupt function
// The name of the sensor object can be changed by defining SENSOR_NAME before including this file
// 
// pigpio's function to set interrupts requires you to pass the function to be called as an argument
// The problem is member functions cannot be passed as an argument in C++
// (No lambdas which capture the current instance of the Sensor class do not work either)
// But any interrupt handler would need access to variables within its instance of the Sensor class (such as the callback function the user provided or how to handle the interrupt)
// 
// Because of this, this solution creates a pointer to the Sensor object immediately without initializing it (must be a pointer otherwise the constructor will be called which would cause other issues)
// The interrupt function is used as a global function that is passed as the pigpio interrupt callback. It calls the interrupt handler of the sensor object depending on what gpio pin was triggered
//
// This is fine because in order for the interrupt function to be called, the sensor object must have been initialized by the user so we shouldnt get a nullptr error
// We just need to make sure the deconstructor of Sensor detaches this function from the interrupt or we will get a nullptr error
//
// Yes I hate this solution as much as everyone else
// Go ahead and fix it if you can
inline Sensor* SENSOR_NAME;
inline void interrupt(int gpio, int level, uint32_t tick){
	SENSOR_NAME->intHandler(gpio, level, tick);
}


class Wheel{
public:

	Wheel(uint8_t address, uint8_t bus);

	void turnWheel(float degrees, uint8_t direction, std::function<void(bool)> callback);
	void resetRotation(std::function<void(bool)> callback);
	void setRotation(float degrees, std::function<void(bool)> callback);
	float getRotation();
	void getRotaitonAsync(std::function<void(bool)> callback);

	void move(float revolutions, std::function<void(bool)> callback);
	float getPosition();
	void getPositionAsync(std::function<void(bool)> callback);
	void drive();
	void stop();



private:
	uint8_t _bus;
	uint8_t _address;
	

}
#endif
