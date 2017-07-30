
//http://www.primrosebank.net/computers/amiga/upgrades/amiga_upgrades_storage_fdis.htm
//http://comp.sys.amiga.hardware.narkive.com/H9WaY49O/external-floppy-drive-for-amiga-500-options

//wire up pin_DiskWriteEnable - currently hardwired to +5v

int pin_DiskReady = 2;        //DB23 - Pin 1
int pin_DiskReadData = 3;     //DB23 - Pin 2
int pin_DiskMotorControl = 4; //DB23 - Pin 8
//int pin_SelectDrive2 = 5;     //DB23 - Pin 9
int pin_DiskReset = 6;        //DB23 - Pin 10
int pin_DiskRemoved = 7;      //DB23 - Pin 11
int pin_SelectDiskSide = 8;   //DB23 - Pin 13
int pin_DiskWriteProtect = 9; //DB23 - Pin 14
int pin_HeadOverTrack0 = 10;  //DB23 - Pin 15
//int pin_DiskWriteEnable = 0; //DB23 - Pin 16
//int pin_DiskWriteData = ?;  //DB23 - Pin 17
int pin_Step = 11;            //DB23 - Pin 18
int pin_HeadDirection = 12;   //DB23 - Pin 19
//int pin_SelectDrive3 = ?;     //DB23 - Pin 20
int pin_SelectDrive1 = 5;     //DB23 - Pin 21
int pin_DiskIndexPulse = 13;     //DB23 - Pin 22

unsigned long motorSpinTime = 1000UL; //in ms
static const int pulseDelayTime = 6;

#define directionToCentre LOW
#define directionToOuter HIGH

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print(x)
 #define DEBUG_PRINTDEC(x)     Serial.print(x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
#endif 

void setup() {
  //setup serial monitor
  Serial.begin(1000000);  
  DEBUG_PRINTLN("Starting setup.");
  delay(1000);

  //setup pins.
  //output
  //pinMode(pin_DiskWriteEnable, OUTPUT);
  pinMode(pin_DiskReset, OUTPUT);
  pinMode(pin_HeadDirection, OUTPUT);
  pinMode(pin_Step, OUTPUT);
  pinMode(pin_DiskMotorControl, OUTPUT);
  pinMode(pin_SelectDrive1, OUTPUT);
  pinMode(pin_SelectDiskSide, OUTPUT);
  

  //input
  pinMode(pin_DiskReady, INPUT_PULLUP);
  pinMode(pin_DiskIndexPulse, INPUT_PULLUP); //Pin is Open Collector
  pinMode(pin_HeadOverTrack0, INPUT_PULLUP);
  pinMode(pin_DiskRemoved, INPUT_PULLUP);
  pinMode(pin_DiskWriteProtect, INPUT_PULLUP);
  pinMode(pin_DiskReady, INPUT_PULLUP);
  pinMode(pin_DiskReadData, INPUT_PULLUP);

  //pin defaults
  //digitalWrite(pin_DiskWriteEnable, HIGH); //disable write mode, ensure read mode
  digitalWrite(pin_DiskReset, HIGH); //pulse low to reset and turn off motors
  digitalWrite(pin_DiskMotorControl, HIGH); //motor off
  selectDiskSide(1);

  //Latch all defaults to the drive
  TurnDF1On();
  TurnDF1Off();

  //As a test, spin the motor for 1 seconds
  spinMotor(1);
  
  //Get ID
  getDriveID();

  stepToCentre();
  //minimum of 18mS when changing direction
  delay(36);
  stepToOutside();

  //waitForIndex();
  readData();
  
  DEBUG_PRINTLN("Finished setup.");
  Serial.println("jfklgdfhlkjdhdlkjhfd");
  
}

void selectDiskSide(bool side){
  if (side){
    digitalWrite(pin_SelectDiskSide, LOW);
    DEBUG_PRINTLN("Side 1");
  }else{
    digitalWrite(pin_SelectDiskSide, HIGH);
    DEBUG_PRINTLN("Side 0");
  }
}

inline void TurnMotorOn(){
  MotorControl(true);
}

inline void TurnMotorOff(){
  MotorControl(false);
}

void MotorControl(bool enabled){
  //start/stop spinning
  TurnDF1Off();
  digitalWrite(pin_DiskMotorControl,!enabled);
  TurnDF1On();
}

void readData() {
  printState("Beginning read");

  //start spinning
  TurnMotorOn();

  printState("Waiting for disk ready.");
  while(digitalRead(pin_DiskReady));
  printState("Disk ready.");
  printState("Waiting for index pulse.");
  //wait for pulse
  while(digitalRead(pin_DiskIndexPulse));
  
  //wait for end of pulse 0
  while(!digitalRead(pin_DiskIndexPulse));
  while(digitalRead(pin_DiskIndexPulse)){
    if (digitalRead(pin_DiskReadData)){
      printState(" Data: 1");
    }else{
      printState(" Data: 0");
    }
  }
  printState("Pulse done.");

  printState("end of waiting for index pin to pulse");

  //stop spinning
  TurnMotorOff();
  TurnDF1Off();
}

void waitForIndex() {
  printState("beginning to wait for index pin to pulse");

  //start spinning
  TurnMotorOn();

  printState("Waiting for disk ready.");
  while(digitalRead(pin_DiskReady));
  printState("Disk ready.");
  printState("Waiting for index pulse.");
  //wait for pulse
  while(digitalRead(pin_DiskIndexPulse));
  //printState("Waiting for end of index pulse.");
  //wait for end of pulse 0
  while(!digitalRead(pin_DiskIndexPulse));
  printState("Pulse done.");

  printState("end of waiting for index pin to pulse");

  //stop spinning
  TurnMotorOff();
  TurnDF1Off();
}

//print the state of the index and track
void printState(const char* charPrint) {
  Serial.print("/RDY ");
  Serial.print(digitalRead(pin_DiskReady));
  Serial.print(" Index:");
  Serial.print(digitalRead(pin_DiskIndexPulse));
  Serial.print(" Track:");
  Serial.print(digitalRead(pin_HeadOverTrack0));
  Serial.print(" ");
  Serial.println(charPrint);
}

void spinMotor(int seconds){
  DEBUG_PRINT("Spinning motor for ");
  DEBUG_PRINT(seconds);
  DEBUG_PRINTLN(" seconds.");
  digitalWrite(pin_DiskMotorControl, LOW); //motor on
  TurnDF1On();
  TurnDF1Off();
  delay(seconds * 1000);
  digitalWrite(pin_DiskMotorControl, HIGH); //motor off
  TurnDF1On();
  TurnDF1Off();
}

//A 32-bit word is read serially from the drive.
void getDriveID(){
  DEBUG_PRINTLN("Getting drive ID.");
  //To start this identification, 
  //the MTR line of the drive in question must be turned on and then off again
  //This resets the serial shift register in the drive. 
  
  TurnDF1Off();
  digitalWrite(pin_DiskMotorControl, LOW);
  TurnDF1On();
  delay(1000);
  TurnDF1Off();
  digitalWrite(pin_DiskMotorControl, HIGH);
  TurnDF1On();
  TurnDF1Off();

  //The individual data bits can then be read by placing the SELx line low and reading 
  //the value of the RDY as a data bit and then placing the SELx line high again.

  //This process is repeated 32 times. The bit first received is the
  //MSB (Most-Significant Bit) of the data word. Since the RDY line is active low, the 
  //data bits must be inverted. 

  unsigned long value = 0;
  for (int i=0;i<32;i++){
    TurnDF1On();
    if (!digitalRead(pin_DiskReady)){
      value |= (1 << i);
    }
    TurnDF1Off();  
  }

  if (value == 0){
    //no drive
    DEBUG_PRINT("No drive connected.");
  }else if (value == 0xFFFFFFFF ){ 
    //standard 3.5 DD
    DEBUG_PRINT("Standard 3 1/2 inch DD drive attached.");
  }else if (value == 0x55555555 ){ 
    // 5.25
    DEBUG_PRINT("Amiga 5 1/4 drive, 2x40 tracks.");
  }else if (value == 0xAAAAAAAA ){ 
    // 3.5 HD
    //IDe from http://comp.sys.amiga.hardware.narkive.com/H9WaY49O/external-floppy-drive-for-amiga-500-options
    DEBUG_PRINT("Amiga 3 1/2 inch HD attached.");
  }else { 
    //unknown
    DEBUG_PRINT("Unknown disk drive ID.");
  }
  
  DEBUG_PRINT(" ID: $");
  DEBUG_PRINTLN(String(value, HEX));

  /*
    ID's used:
    $0000 0000 No drive connected (00)
    $FFFF ffff Standard Amiga 3 1/2" drive (11)
    $5555 5555 Amiga5 1/4" drive, 2x40 tracks (01)
    $AAAA AAAA - IIRC, this was the code for the HD 3.5" floppy (
  */
}

void TurnDF1On() {
  digitalWrite(pin_SelectDrive1, LOW);
  //DEBUG_PRINTLN("DF1 turned on (signal low)");
  delay(1);
}

void TurnDF1Off() {
  digitalWrite(pin_SelectDrive1, HIGH);
  //DEBUG_PRINTLN("DF1 turned off (signal high)");
  delay(1);
}

//pulse the step pin
void pulseStep() {
  //TurnDF1On();
  digitalWrite(pin_Step,LOW);
  delay(pulseDelayTime);
  digitalWrite(pin_Step,HIGH);
  //TurnDF1Off();
}

//Step the head out
void moveHeadOut() {
  //Signal used is quick pulse (high, low, then high).

  //If we are over track zero do not allow stepping out any more
  if (digitalRead(pin_HeadOverTrack0)){
    digitalWrite(pin_HeadDirection, directionToOuter);
    pulseStep();
    DEBUG_PRINTLN("Moving heads outwards.");    
  }else{
    //We are over track zero
    DEBUG_PRINTLN("Over Track 0 - NOT moving the heads.");    
  }
}

//Step the head out
void moveHeadIn() {
  //Signal used is quick pulse (high, low, then high).
  digitalWrite(pin_HeadDirection, directionToCentre);
  pulseStep();
  DEBUG_PRINTLN("Moving heads to the centre.");    
}

void stepToOutside() {
  TurnDF1On();
  for(int i=0;i<100;i++) {
      moveHeadOut();
  }
  TurnDF1Off();
}

void stepToCentre() {
  TurnDF1On();
  for(int i=0;i<100;i++) {
      moveHeadIn();
  }
  TurnDF1Off();
}

void loop() {
  // put your main code here, to run repeatedly:
}

