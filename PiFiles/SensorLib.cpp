#include "SensorLib.h"

Sensor::Sensor(uint8_t address, uint8_t bus){
	_addr = address;
	_bus = bus;
	constructorUni();
}

Sensor::Sensor(){
	_addr = 0x19;
	_bus = 1;
	constructorUni();
}

// Deconstructor removes the interrput to avoid nullptr errors
Sensor::~Sensor(){
	gpioSetAlertFunc(INT_PIN, NULL);
}

void Sensor::constructorUni(){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}
	
	// Read data
	uint8_t data = i2cReadByteData(handle, _handshakeRegister);

	// Close I2C bus
	i2cClose(handle);

	// Make sure data is correct
	if(data != _addr){
		std::cerr << "Sensor handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_addr << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}

	// Tell user everything is ok
	std::cout << "Succesfully connected to sensors!" << std::endl;

	// Set interrupt
	gpioSetAlertFunc(INT_PIN, interrupt);

}

int8_t Sensor::getAngle(float angle, std::function<void(RangeFinderPacket&)> callbackFcn){
	// Make sure the arduino is free
	if(_interruptState != INT_STATE_NONE){
		return ERROR_BUSY;
	}

	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	// Send command
	i2cWriteI2CBlockData(handle, _angleRegister, (char*)(&angle), sizeof(angle));

	// Close I2C transmission
	i2cClose(handle);

	// Set interrupt state
	_interruptState = INT_STATE_GET_ANGLE;

	_angleCallback = callbackFcn;

	return 0;
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn){
	// Make sure the arduino is free
	if(_interruptState != INT_STATE_NONE){
		return ERROR_BUSY;
	}
	
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	// Send command
	i2cWriteByte(handle, _scanRegister);

	// Close I2C transmission
	i2cClose(handle);

	_interruptState = INT_STATE_SCAN;

	_scanCallback = callbackFcn;

	return 0;

}

void Sensor::intHandler(int gpio, int level, uint32_t tick){
	// Return if not rising edge
	if(level != 1) return;

	switch(_interruptState){
	case INT_STATE_GET_ANGLE:{
		RangeFinderPacket data;
		int8_t handle = i2cOpen(_bus, _addr, 0);

		if(handle < 0){
			std::cerr << "Unable to connect to sensor" << std::endl;
			return;
		}

		// Read data into data variable
		i2cReadI2CBlockData(handle, _readAngleRegister, (char*)(&data), 6);


		i2cClose(handle);

		_interruptState = INT_STATE_NONE;

		_angleCallback(data);

		break;
	}
	case INT_STATE_SCAN:{
		std::vector<RangeFinderPacket> scanData;

		int8_t handle = i2cOpen(_bus, _addr, 0);

		if(handle < 0){
			std::cerr << "Unable to connect to sensor" << std::endl;
			return;
		}

		for(uint8_t i = 0; i < (uint8_t)ceil(SCAN_POINTS*6/32.0f); i++){
			uint8_t bufferSize = 32/6;
			RangeFinderPacket data[bufferSize];

			i2cReadI2CBlockData(handle, _readScanRegister, (char*)(&data), bufferSize*6);

			for(int j = 0; j < bufferSize; j++){
				if(scanData.size() >= SCAN_POINTS) break;

				scanData.push_back(data[j]);
			}
		}	

		i2cClose(handle);

		_interruptState = INT_STATE_NONE;

		_scanCallback(scanData);

		break;
	}

	}

}

float Sensor::getHeading(){
	float output;
	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 0;
	}

	i2cReadI2CBlockData(handle, _headingRegister, (char*)(&output), sizeof(float));

	i2cClose(handle);

	return output;
}

uint8_t Sensor::getRSSI(){
	uint8_t output;
	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 255;
	}

	i2cReadI2CBlockData(handle, _rssiRegister, (char*)(&output), 1);

	i2cClose(handle);

	return output;
}

void Sensor::getHeadingRSSI(float& heading, uint8_t& rssi){
	heading = getHeading();
	rssi = getRSSI();
}








Wheel::Wheel(uint8_t address, uint8_t bus, uint8_t interruptPin){
	_address = address;
	_bus = bus;
	Wheel::interruptPin = interruptPin;


	uint8_t data = *(uint8_t*)readData(HANDSHAKE_REGISTER, 1);

	// Make sure data is correct
	if(data != _address){
		std::cerr << "Wheel handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_address << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}

	// Tell user everything is ok
	std::cout << "Succesfully connected to wheel at " << (int)address << std::endl;

	// Set interrupt
	gpioSetAlertFunc(interruptPin, interrupt);
}

Wheel::~Wheel(){
	gpioSetAlertFunc(interruptPin, NULL);
}

void Wheel::writeData(uint8_t reg, void* data, uint8_t length){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteI2CBlockData(handle, reg, (char*)data, length);

	// Close I2C bus
	i2cClose(handle);
}

void Wheel::writeRegister(uint8_t reg){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteByte(handle, reg);

	// Close I2C bus
	i2cClose(handle);
}

void Wheel::writeRegister(uint8_t reg, uint8_t data){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteByteData(handle, reg, data);

	// Close I2C bus
	i2cClose(handle);
}
	

void* Wheel::readData(uint8_t reg, uint8_t length){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return nullptr;
	}
	
	char* data = new char[length];
	
	// Read data
	i2cReadI2CBlockData(handle, reg, data, length);

	// Close I2C bus
	i2cClose(handle);

	return (void*)data;

}


int8_t Wheel::turnWheel(float degrees, std::function<void(int8_t)> callback){
	if(_state == INT_STATE_NONE) return ERROR_BUSY;

	degrees = degrees/360;

	writeData(TURN_REGISTER, &degrees, sizeof(degrees));
	
	intCallback = callback;
	_state = INT_STATE;

	return 0;
}

int8_t Wheel::resetRotation(std::function<void(int8_t)> callback){
	if(_state == INT_STATE_NONE) return ERROR_BUSY;

	writeRegister(RESET_ROTATION_REGISTER);

	intCallback = callback;
	_state = INT_STATE;

	return 0;
}



int8_t Wheel::setRotation(float degrees, std::function<void(int8_t)> callback){
	if(_state == INT_STATE_NONE) return ERROR_BUSY;

	degrees = degrees/360;

	writeData(SET_ROTATION_REGISTER, &degrees, sizeof(degrees));

	intCallback = callback;
	_state = INT_STATE;

	return 0;
}


float Wheel::getRotation(){
	return *(float*)readData(GET_ROTATION_REGISTER, sizeof(float));
}


int8_t Wheel::move(float revolutions, std::function<void(int8_t)> callback){
	if(_state == INT_STATE_NONE) return ERROR_BUSY;

	writeData(MOVE_REGISTER, &revolutions, sizeof(revolutions));

	intCallback = callback;
	_state = INT_STATE;

	return 0;
}


void Wheel::drive(){
	writeRegister(DRIVE_REGISTER);
}

void Wheel::stop(){
	writeRegister(STOP_REGISTER);
}

float Wheel::getPosition(){
	return *(float*)readData(GET_POSITION_REGISTER, sizeof(float));
}


void Wheel::intHandler(int gpio, int level, uint32_t tick){
	// Make sure we are expecting data
	if(_state == INT_STATE_NONE) {
		// If we arent expecting data just read the register to clear the interrupt and exit the function
		readData(RESPONSE_REGISTER, 1);
		return;
	}

	// Clear the current state
	_state = INT_STATE_NONE;

	// Call the callback function with the data recieved
	intCallback(*(int8_t*)readData(RESPONSE_REGISTER, 1));
}

