#include <Arduino.h>
#include <Wire.h>

#define ADDRESS 0x19

#define HANDSHAKE_REGISTER 0x01
#define TURN_REGISTER 0x02
#define RESET_ROTATION_REGISTER 0x03
#define SET_ROTATION_REGISTER 0x04
#define GET_ROTATION_REGISTER 0x05
#define MOVE_REGISTER 0x06
#define DRIVE_REGISTER 0x07
#define STOP_REGISTER 0x08
#define GET_POSITION_REGISTER 0x09
#define RESPONSE_REGISTER 0x0A

#define DRIVE_ENCODER1 2
#define DRIVE_ENCODER2 7
#define TURN_ENCODER1 3
#define TURN_ENCODER2 8

#define DRIVE_MOTOR1 5
#define DRIVE_MOTOR2 6
#define TURN_MOTOR1 9
#define TURN_MOTOR2 10

#define TURN_TICKS_PER_REVOLUTION 1000
#define DRIVE_TICKS_PER_REVOLUTION 10200

#define INT_PIN 4

#define LIMIT_PIN 12

#define STATE_NONE 0
#define TURN_STATE_TURN 1
#define TURN_STATE_RESET 2
#define TURN_STATE_SET 3
#define DRIVE_STATE_MOVE 1
#define DRIVE_STATE_DRIVE 2

#define DRIVE_ERROR 10
#define TURN_ERROR 10

#define dist(A, B, N) A - B > N - (A - B - 1) ? min(A - B, N - (A - B - 1)) : -min(A - B, N - (A - B - 1))

uint8_t driveState = STATE_NONE;
uint8_t turnState = STATE_NONE;

uint8_t* response;
uint8_t responseLength = 0;
uint8_t ok = 0;

int16_t targetTurn = 0;
int32_t targetDrive = 0;

int32_t driveMotorPosition = 0;
int16_t turnMotorPosition = 0;

void driveEncoderISR();
void turnEncoderISR();
void receiveEvent(int count);
void requestEvent();

void setup() {
  // Set pins
  pinMode(DRIVE_ENCODER1, INPUT);
  pinMode(DRIVE_ENCODER2, INPUT);
  pinMode(TURN_ENCODER1, INPUT);
  pinMode(TURN_ENCODER2, INPUT);

  pinMode(DRIVE_MOTOR1, OUTPUT);
  pinMode(DRIVE_MOTOR2, OUTPUT);
  pinMode(TURN_MOTOR1, OUTPUT);
  pinMode(TURN_MOTOR2, OUTPUT);

  pinMode(INT_PIN, OUTPUT);

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  // Begin serial interfaces
  Wire.begin();
  Serial.begin(9600);
  
  // Enable interrupts
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENCODER1), driveEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(TURN_ENCODER1), turnEncoderISR, RISING);

  // Set I2C handling functions
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

// This method has a target position for the drive and turn motors
// Every loop it checks if the motors are are their respective target positions
// If they arent at their respective positions, move them towards them
void loop() {
  // Make sure the turn motor doesnt exceed the bounds of -180deg to 180deg
  if(turnMotorPosition > TURN_TICKS_PER_REVOLUTION/2){
    turnMotorPosition -= TURN_TICKS_PER_REVOLUTION;
  }else if(turnMotorPosition < -TURN_TICKS_PER_REVOLUTION/2){
    turnMotorPosition += TURN_TICKS_PER_REVOLUTION;
  }

  // Handle turn motor
  if(turnState == TURN_STATE_RESET){
    // Turn until the limit switch is triggered
    // Reset current and target positions when limit switch is triggered
    if(LIMIT_PIN){
      digitalWrite(TURN_MOTOR1, LOW);
      digitalWrite(TURN_MOTOR2, LOW);
      
      turnMotorPosition = 0;
      targetTurn = 0;

      // Trigger interrupt to tell pi the operation was completed
      ok = 1;
      digitalWrite(INT_PIN, HIGH);
      turnState = STATE_NONE;
    }else{
      // Havent hit limit switch yet, keep turning
      digitalWrite(TURN_MOTOR1, HIGH);
      digitalWrite(TURN_MOTOR2, LOW);
    }

  }else if(dist(turnMotorPosition, targetTurn, TURN_TICKS_PER_REVOLUTION) > TURN_ERROR){
    // Target position is closest if we move in the positive direction
    digitalWrite(TURN_MOTOR1, HIGH);
    digitalWrite(TURN_MOTOR2, LOW);

  }else if(dist(turnMotorPosition, targetTurn, TURN_TICKS_PER_REVOLUTION) < -TURN_ERROR){
    // Target position is closest if we move in the negative direction
    digitalWrite(TURN_MOTOR1, LOW);
    digitalWrite(TURN_MOTOR2, HIGH);

  }else{
    // If we just reached the target position, tell the Pi everything is ok
    if(turnState != STATE_NONE){
      ok = 1;
      digitalWrite(INT_PIN, HIGH);
      turnState = STATE_NONE;
    }

    // Make sure the motor is stopped
    digitalWrite(TURN_MOTOR1, LOW);
    digitalWrite(TURN_MOTOR2, LOW);
  }


  // Handle the drive motor
  if(driveState == DRIVE_STATE_DRIVE){
    // Just drive forward if told to drive
    digitalWrite(DRIVE_MOTOR1, HIGH);
    digitalWrite(DRIVE_MOTOR2, LOW);

  }else if(driveMotorPosition < targetDrive - DRIVE_ERROR){
    // Target position is closest if we move in the positive direction
    digitalWrite(DRIVE_MOTOR1, HIGH);
    digitalWrite(DRIVE_MOTOR2, LOW);

  }else if(driveMotorPosition > targetDrive + DRIVE_ERROR){
    // Target position is closest if we move in the negative direction
    digitalWrite(DRIVE_MOTOR1, LOW);
    digitalWrite(DRIVE_MOTOR2, HIGH);

  }else{
    // If we just reached the target position, tell the Pi everything is ok
    if(driveState != STATE_NONE){
      ok = 1;
      digitalWrite(INT_PIN, HIGH);
      driveState = STATE_NONE;
    }

    // Make sure the motor is off if at the target position
    digitalWrite(DRIVE_MOTOR1, LOW);
    digitalWrite(DRIVE_MOTOR2, LOW);
  }
}


// Handle I2C data
void receiveEvent(int count){
  // Save register
  uint8_t reg = Wire.read();

  switch(reg){
  case HANDSHAKE_REGISTER:{
    // Handshake just responds with the device address to verify communication is working
    response = new uint8_t(ADDRESS);
    responseLength = 1;
    break;
  }

  case GET_ROTATION_REGISTER:{
    // Respond with current rotation in degrees
    float data = ((float)turnMotorPosition)/(TURN_TICKS_PER_REVOLUTION*360);
    response = (uint8_t*)new float(data);
    responseLength = sizeof(data);
    break;
  }

  case GET_POSITION_REGISTER:{
    // Respond with current position in revolutions
    float data = ((float)driveMotorPosition)/DRIVE_TICKS_PER_REVOLUTION;
    response = (uint8_t*)new float(data);
    responseLength = sizeof(data);
    break;
  }

  case RESPONSE_REGISTER:{
    // Tells the Pi if previous operation completed successfully
    response = new uint8_t(ok);
    responseLength = 1;
    ok = 0;

    // Clear interrupt line
    digitalWrite(INT_PIN, LOW);
    break;
  }

  case TURN_REGISTER:{
    // Turn the wheel a specified number of degrees
    turnState = TURN_STATE_TURN;
    float data;
    for(uint8_t i = 0; i < sizeof(float); i++) ((uint8_t*)&data)[i] = Wire.read();
    targetTurn = turnMotorPosition + data*TURN_TICKS_PER_REVOLUTION;

    // Make sure the target rotation is within the bounds of -180deg to 180deg
    while(abs(targetTurn) > TURN_TICKS_PER_REVOLUTION/2){
      if(targetTurn > TURN_TICKS_PER_REVOLUTION/2){
        targetTurn -= TURN_TICKS_PER_REVOLUTION;
      }else{
        targetTurn += TURN_TICKS_PER_REVOLUTION;
      }
    }
    break;
  }

  case RESET_ROTATION_REGISTER:{
    // Set the state to reset the rotation of the wheel
    turnState = TURN_STATE_RESET;
    break;
  }

  case SET_ROTATION_REGISTER:{
    // Set an absolute rotation to move to
    turnState = TURN_STATE_SET;
    float data;
    for(uint8_t i = 0; i < sizeof(float); i++) ((uint8_t*)&data)[i] = Wire.read();
    targetTurn = data*TURN_TICKS_PER_REVOLUTION;

    // Make sure the target rotation is within the bounds of -180deg to 180deg
    while(abs(targetTurn) > TURN_TICKS_PER_REVOLUTION/2){
      if(targetTurn > TURN_TICKS_PER_REVOLUTION/2){
        targetTurn -= TURN_TICKS_PER_REVOLUTION;
      }else{
        targetTurn += TURN_TICKS_PER_REVOLUTION;
      }
    }
    break;
  }

  case MOVE_REGISTER:{
    // Move the wheel a specified number of revolutions
    driveState = DRIVE_STATE_MOVE;
    float data;
    for(uint8_t i = 0; i < sizeof(float); i++) ((uint8_t*)&data)[i] = Wire.read();
    targetDrive = driveMotorPosition + data*DRIVE_TICKS_PER_REVOLUTION;
    break;
  }

  case DRIVE_REGISTER:{
    // Turn wheel on
    driveState = DRIVE_STATE_DRIVE;
    digitalWrite(DRIVE_MOTOR1, HIGH);
    digitalWrite(DRIVE_MOTOR2, LOW);
    break;
  }

  case STOP_REGISTER:{
    // Stop wheel
    driveState = STATE_NONE;
    digitalWrite(DRIVE_MOTOR1, LOW);
    digitalWrite(DRIVE_MOTOR2, LOW);
    break;
  }
  }
}

// Responds with whatever is stored at the location pointed to by the response pointer
void requestEvent(){
  Wire.write(response, responseLength);
  for(uint8_t i = 0; i < responseLength; i++){
    delete (response + i);
  }
  responseLength = 0;
}

// Increments/decrements the position of the drive motor
void driveEncoderISR(){
  if(digitalRead(DRIVE_ENCODER2)){
    driveMotorPosition++;
  }else{
    driveMotorPosition--;
  }
}

// Increments/decrements the position of the turn motor
void turnEncoderISR(){
  if(digitalRead(TURN_ENCODER2)){
    turnMotorPosition++;
  }else{
    turnMotorPosition--;
  }
}