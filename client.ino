/*
B.I.K.E. Helmet V2
By: Jeremy Dunne 
ISEF 2016 

Takes information recieved from B.I.K.E. Control Program and warns cyclist via display on helmet 


*/


#define RX_PIN 8
#define TX_PIN 7
#define SERVER "192.168.43.115"
#define NETWORK_SSID "Zorag"
#define PASSWORD "daisyDaisy"
#define PORT 8087
#define RIGHT_ARROW_LOCATION
#define LEFT_ARROW_LOCATION
#define READY_FOR_INFORMATION_COMMAND "RDY"
#define DEBUG false
#define DEBUG_HEADING 240
#define STANDARD_RESPONSE_TIMEOUT 5000 
#define BUTTON 5 
#define JOIN_NETWORK_COMMAND_PREFIX "AT+CWJAP="
#define SUT_UP_TCP_PREFIX "AT+CIPSTART=\"TCP\","
#define SEND_INFO_PREFIX "AT+CIPSEND="
#define MAX_DEVICE_COMMAND "AT+CIPMUX=1"
#define SHORT_DELAY_TIME 1000
#define QUICK_DELAY_TIME 500
#define ID_STATEMENT "ID,BICYCLE" 
#define HEADING_PREFIX "Comp," 
#define SHOULD_ASK_FOR_INTERSECTION_NAME true 
#define INTERSECTION_REQUEST "IR" 


#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include <Wire.h> //I2C Arduino Library


int headingLoc[2] = {0, 0}; 
const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address of HMC5883
const byte hmc5883ModeRegister      = 0x02;
const byte hmcContinuousMode        = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;
bool isLeftSlowShown; 
bool isRightSlowShown; 
int initLoc[2] = {0, 120}; 
int intersectionLoc[2] = {0, 110}; 
int card[2] = {60, 40}; 
bool hasInitCleared = false; 
char leftArrowShown; 
char rightArrowShown;
String lastCardinal; 
String intersection; 
int search[2] = {0, 100}; 
// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    -1  
#define TFT_DC     9
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

int demoHeadings[3] = {225,315,45};
int demoMode = 1; 
//esp8266

SoftwareSerial wifi(RX_PIN,TX_PIN);


#define HMC58831_REGISTER_CONFIGURATION_A   0x00
#define HMC58831_REGISTER_CONFIGURATION_B   0x01
#define HMC58831_REGISTER_READ_MODE         0x02
#define HMC58831_REGISTER_DATA_X_MSB        0x03
#define HMC58831_REGISTER_DATA_X_LSB        0x04
#define HMC58831_REGISTER_DATA_Y_MSB        0x07
#define HMC58831_REGISTER_DATA_Y_LSB        0x08
#define HMC58831_REGISTER_DATA_Z_MSB        0x05
#define HMC58831_REGISTER_DATA_Z_LSB        0x06
#define HMC58831_REGISTER_STATUS            0x09
#define HMC58831_REGISTER_ID_A              0x0A
#define HMC58831_REGISTER_ID_B              0x0B
#define HMC58831_REGISTER_ID_C              0x0C


enum hmc58831_data_rate {
  HMC5883L_DATARATE_75HZ       = 0b110,
  HMC5883L_DATARATE_30HZ       = 0b101,
  HMC5883L_DATARATE_15HZ       = 0b100,
  HMC5883L_DATARATE_7_5HZ      = 0b011,
  HMC5883L_DATARATE_3HZ        = 0b010,
  HMC5883L_DATARATE_1_5HZ      = 0b001,
  HMC5883L_DATARATE_0_75_HZ    = 0b000
};

enum hmc58831_read_mode {
  HMC58831_CONTINUOUS             = 0x00,
  HMC58831_SINGLE                 = 0x01,
  HMC58831_IDLE                   = 0x02
};

enum hmc58831_range{
  HMC5883L_RANGE_8_1GA     = 0b111,
  HMC5883L_RANGE_5_6GA     = 0b110,
  HMC5883L_RANGE_4_7GA     = 0b101,
  HMC5883L_RANGE_4GA       = 0b100,
  HMC5883L_RANGE_2_5GA     = 0b011,
  HMC5883L_RANGE_1_9GA     = 0b010,
  HMC5883L_RANGE_1_3GA     = 0b001,
  HMC5883L_RANGE_0_88GA    = 0b000
};

enum hmc58831_samples_per_read {
  HMC5883L_SAMPLES_1            = 0x00,
  HMC5883L_SAMPLES_2            = 0x01,
  HMC5883L_SAMPLES_4            = 0x10,
  HMC5883L_SAMPLES_8            = 0x11
};







enum hmc58831_self_test{
  HMC5883L_SELF_TEST_POSITIVE = 0x01,
  HMC5883L_SELF_TEST_NEGATIVE = 0x02,
  HMC5883L_SELF_TEST_NONE = 0x00
};


struct rawMagData{
  int x = 0;
  int y = 0;
  int z = 0;
};

struct magData{
  float heading = 0;
  rawMagData rawData;
};


class HMC5883L{
  public:
    int begin();
    void setMode(hmc58831_read_mode mode);
    void setSamplesPerRead(hmc58831_samples_per_read samples);
    void setDataRange(hmc58831_range range);
    void setDataRate(hmc58831_data_rate rate);
    rawMagData getRawData();
    magData getMagData();
    void beginSelfTest(hmc58831_self_test test);
    void endSelfTest();
    void setAxisOffsets(int x, int y, int z);
    void setAxisScales(double x, double y, double z);
  private:
    void write8(int reg, int value);
    void write8(int reg);
    int read8(int reg);
    int read16(int reg);
    bool checkDevice();
    int xOffset = 0;
    double xScale = 1.0;
    int yOffset = 0;
    double yScale = 1.0;
    int zOffset = 0;
    double zScale = 1.0;
    float calculateHeading(float x, float y);
    const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address of HMC5883

};




rawMagData HMC5883L::getRawData(){
  rawMagData data;
  data.x = (read16(HMC58831_REGISTER_DATA_X_MSB) - xOffset) * xScale;
  data.z = (read16(HMC58831_REGISTER_DATA_Z_MSB) - zOffset) * zScale;
  data.y = (read16(HMC58831_REGISTER_DATA_Y_MSB) - yOffset) * yScale;
  return data;
}

magData HMC5883L::getMagData(){
  magData data;
  data.rawData = getRawData();
  data.heading = calculateHeading(data.rawData.x, data.rawData.y);
  return data;
}

void HMC5883L::beginSelfTest(hmc58831_self_test test){
  int value = read8(HMC58831_REGISTER_CONFIGURATION_A);
  value &= 0b11111100;
  value |= test;
  write8(HMC58831_REGISTER_CONFIGURATION_A,value);
  delay(10);
  //read 3 times to let the mag update
  for(int i = 0; i < 3; i ++){
    getRawData();
    delay(5);
  }
}

void HMC5883L::setAxisOffsets(int x, int y, int z){
  xOffset = x;
  yOffset = y;
  zOffset = z;
}

void HMC5883L::setAxisScales(double x, double y, double z){
  xScale = x;
  yScale = y;
  zScale = z;
}

void HMC5883L::endSelfTest(){
  int value = read8(HMC58831_REGISTER_CONFIGURATION_A);
  value &= 0b11111100;
  value |= HMC5883L_SELF_TEST_NONE;
  write8(HMC58831_REGISTER_CONFIGURATION_A,value);
  delay(10);
  //read 3 times to let the mag update
  for(int i = 0; i < 3; i ++){
    getRawData();
    delay(5);
  }
}

float HMC5883L::calculateHeading(float x, float y){
  float heading = atan2(y , x) *180.0 / M_PI;
  if(heading < 0) heading += 360;
  else if(heading > 360) heading -= 360;
  return heading;
}



int HMC5883L::begin(){
  
  if(!checkDevice()){
    return -1;
  }
  setDataRange(HMC5883L_RANGE_1_3GA);
  //beginSelfTest(HMC5883L_SELF_TEST_NEGATIVE);
  setMode(HMC58831_CONTINUOUS);
  setSamplesPerRead(HMC5883L_SAMPLES_8);
  setDataRate(HMC5883L_DATARATE_30HZ);
  return 0;
}

void HMC5883L::setMode(hmc58831_read_mode mode){
  int value = read8(HMC58831_REGISTER_READ_MODE);
  value &= 0b11111100;
  value |= mode;
  write8(HMC58831_REGISTER_READ_MODE, value);
}

void HMC5883L::setSamplesPerRead(hmc58831_samples_per_read samples){
  int value = read8(HMC58831_REGISTER_CONFIGURATION_A);
  value &= 0b10011111;
  value |= (samples << 5);
  write8(HMC58831_REGISTER_CONFIGURATION_A,value);
}

void HMC5883L::setDataRange(hmc58831_range range){
  write8(HMC58831_REGISTER_CONFIGURATION_B,range << 5);
}

void HMC5883L::setDataRate(hmc58831_data_rate rate){
  int value = read8(HMC58831_REGISTER_CONFIGURATION_A);
  value &= 0b11100011;
  value |= (rate << 2);
  write8(HMC58831_REGISTER_CONFIGURATION_A,value);
}


void HMC5883L::write8(int reg, int value){
  Wire.beginTransmission(hmc5883Address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void HMC5883L::write8(int reg){
  Wire.beginTransmission(hmc5883Address);
  Wire.write(reg);
  Wire.endTransmission();
}

int HMC5883L::read8(int reg){
  write8(reg);
  Wire.requestFrom(hmc5883Address,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int HMC5883L::read16(int regLow){
  write8(regLow);
  Wire.requestFrom(hmc5883Address,2);
  int value = -1;
  if(Wire.available() >= 2){
    value = Wire.read();
    value = value << 8;
    value |= Wire.read();
  }
  return value;
}


bool HMC5883L::checkDevice(){
  if(read8(HMC58831_REGISTER_ID_A) != 0x48 || read8(HMC58831_REGISTER_ID_B) != 0x34 || read8(HMC58831_REGISTER_ID_C) != 0x33){
    Serial.println("Found: " + String(read8(HMC58831_REGISTER_ID_A)) + " Expected 0x33");
    return false;
  }
  
  return true;
}


HMC5883L mag; 

int carsLeft[2] = {false};
int carsRight[2] = {false};
bool isImageShown[2] = {false};
bool isInit = false; 

void setup() {
  Wire.begin();
  delay(1000);
  Serial.begin(9600);
 
  pinMode(BUTTON,INPUT_PULLUP); 
  Serial.println("Initialized");
  setUpDisplay();
  //if(digitalRead(BUTTON)){
    //displayCardinal(); 
    //while(true){
    //  delay(300); 
    //  displayCardinal(); 
    ///   int p = getHeading(); 
    //}
  //}
   if(mag.begin() != 0){
    Serial.println("Mag init failed!"); 
    while(true);
  }
  mag.setAxisOffsets(0,-360,6);
  displayCardinal(); 
  //displayGraphicIntersection();
  displayInitStatus(false); 
  
  setupBIKEHelmet();
  
  int p = getHeading(); 
  displayInitStatus(true); 
  displaySearching(); 
  waitForButton(); 
  hideSearch(); 
  isInit = true;
  intersection = getIntersection(); 
  Serial.println("Intersection name: " + intersection);
  clearIntersection(); 
  displayGraphicIntersection();
}

void waitForButton(){
  while(!digitalRead(BUTTON)){
    delay(300); 
    displayCardinal(); 
  }
}

int getHeading(){
  magData data; 
  data = mag.getMagData();
  Serial.println("Heading: " + String((int)(data.heading + .5)));
  displayHeading((int)(data.heading + .5));
  
  return demoHeadings[demoMode];
}

/*
int getHeading(){
  int heading = 0;
  int x,y,z; //triple axis data
  if(DEBUG){
    heading = DEBUG_HEADING;
  }
  else{
      Wire.beginTransmission(hmc5883Address);
      Wire.write(hmcDataOutputXMSBAddress); //select register 3, X MSB register
      Wire.endTransmission();
      Wire.requestFrom(hmc5883Address, 6);
      if(6<=Wire.available()){
        x = Wire.read()<<8; //X msb
        x |= Wire.read(); //X lsb
        z = Wire.read()<<8; //Z msb
        z |= Wire.read(); //Z lsb
        y = Wire.read()<<8; //Y msb
        y |= Wire.read(); //Y lsb
  }
  Serial.println("X: " + String(x) + " Y:" + String(y));  
  heading = float(atan2(y , x)) / M_PI * 180.0; // angle is atan(-y/x)
  if(heading < 0)
     heading = heading  + 360; // angle from 0 to 359 instead of plus/minus 180
  }
  displayHeading((int)heading); 
  Serial.println("Heading: " + String((int)heading));
  return heading;
}
*/

void setUpDisplay(){
  tft.initR(INITR_BLACKTAB); 
  tft.fillScreen(ST7735_BLACK);
}

void initMag(){
  
  /*
  Wire.begin(); 
  int value;
  Wire.beginTransmission(hmc5883Address);
  Wire.write(hmc5883ModeRegister); //select mode register
  Wire.endTransmission(); 
  Wire.requestFrom(hmc5883ModeRegister, 1); 
  if(Wire.available()) value = Wire.read();
  Wire.beginTransmission(hmc5883Address); //open communication with HMC5883
  Wire.write(hmc5883ModeRegister); //select mode register
  value &=  0b11111100;
  value |= hmcContinuousMode;
  Wire.write(value);   //continuous measurement mode
  Wire.endTransmission();


  Wire.beginTransmission(hmc5883Address);
  Wire.write(0x00); //select mode register
  Wire.endTransmission(); 
  Wire.requestFrom(hmc5883ModeRegister, 1); 
  if(Wire.available()) value = Wire.read();
  Wire.beginTransmission(hmc5883Address); //open communication with HMC5883
  Wire.write(0x00); //select mode register
  value &=  0b10011111;
  value |= (0x11 << 5);
  Wire.write(value);   //continuous measurement mode
  Wire.endTransmission();
  */
}

void setupBIKEHelmet(){
	wifi.begin(9600);
	joinNetwork();
  waitForOK(); 
  initMag(); 
  //String resp = readLineFromServer(STANDARD_RESPONSE_TIMEOUT); 
  // Serial.println("RESPONSE: " + resp); 
	joinServer();
  waitForOK(); 
  sendBikeID();
  String resp = readLineFromServer(500); 
  Serial.println("RESPONSE: " + resp);
  sendHeading();
  resp = readLineFromServer(500); 
  Serial.println("RESPONSE: " + resp);
}

String readLineFromServer(long timeout){
	String msg;
  long startMillis = millis(); 
	while(!wifi.available()){
		delay(100);
	}
	delay(1000);
	while(millis() - startMillis < timeout){
  	if(wifi.available()){
  		msg += (char)wifi.read();
  	}
	}
  Serial.println("MESSAGE: " + msg); 
  return msg;
}

void waitForOK(){
  String msg; 
  while(!wifi.available());
  bool isDone = false; 
  while(!isDone){
    if(wifi.available()){
      char m = wifi.read(); 
      Serial.print(m); 
      msg += m; 
    }
    if(msg.indexOf("OK") > 0){
      isDone = true; 
    } 
  }
}

String getServerResponse(){
  //reads until after "OK" recieved 
  bool isDone = false; 
  bool shouldRead = false; 
  String realMsg;
  String totalMsg;  
  bool l = false; 
  while(l == false){
    if(wifi.available()){
      totalMsg += (char)wifi.read(); 
    }
    if(totalMsg.indexOf("OK") > 0)
    l = true; 
  }
  while(wifi.available()){
    realMsg += (char)wifi.read();  
    delay(3); 
  }
  Serial.println("Recieved with junk: " + totalMsg + realMsg); 
  return realMsg; 
}
  


void sendInfoRequest(){
  sendData(READY_FOR_INFORMATION_COMMAND);
}
void showCarsRight(bool state, int mode){
  //0 fast, 1 slow
  bool shouldShowRight = false;
  if(state == true){
   shouldShowRight = true; 
    if(isImageShown[1]){
          //show image
          shouldShowRight = false;
        }
        else {
          //kill image
       
          shouldShowRight = true;
        }
  }
    else {
      if(isImageShown[1]){
        shouldShowRight = false;
      }

  }
  if(shouldShowRight == true){
    //truely show image
    isImageShown[1] = true; 
    if(mode == 0){
       drawRightArrow("Red"); 
    }
    else{
      drawRightArrow("Yellow"); 
    }
    
  }
  if(shouldShowRight == false){
    //kill image here.
    if((carsRight[0] == 1) & (carsRight[1] == 1)){
    isImageShown[1] = false; 
    drawRightArrow("Black");
    }
  }
 
}

void showCarsLeft(bool state, int mode){
  //0 fast, 1 slow
  bool shouldShowLeft = false;

  if(state == true){
        if(isImageShown[0]){
        
          //show image
          shouldShowLeft = false;
        }
        else {
          //kill image
       
          shouldShowLeft = true;
        }  
  }
    if(state == false) {
      if(isImageShown[0]){
        shouldShowLeft = false;
      }

  }
  if(shouldShowLeft == false){
     if((carsLeft[0] == 1) & (carsLeft[1] == 1)){
    //kill image here.
    isImageShown[0] = false; 
    Serial.println("Not Showing image");
    drawLeftArrow("Black");
  }
  }
  if(shouldShowLeft == true){
   
    //truely show image
    isImageShown[0] = true; 
    Serial.println("Showing Left with mode: " + String(mode));
    if(mode == 0){
        drawLeftArrow("Red");
        
    }
    else{drawLeftArrow("Yellow"); }
     
  }
  
  
   
}

void showLeft(){
  Serial.println("Car left pos: " + String(carsLeft[0]) + " " + String(carsLeft[1]));
  if(carsLeft[0] == 0) {
     if(leftArrowShown != 'F'){
     drawLeftArrow("Red");
      leftArrowShown= 'F'; 
    }
  }
  else if(carsLeft[1] == 0){
   if(leftArrowShown != 'S'){
    drawLeftArrow("Yellow");
    leftArrowShown = 'S'; 
    }
  }
  else{
    if(leftArrowShown != 'B'){ 
    drawLeftArrow("Black");
      leftArrowShown = 'B'; 
    }
  }
 
}

void displayInfo(){
  int leftMode,rightMode; 
 showLeft();
  if(carsRight[0] == 0) {
    if(rightArrowShown != 'F'){
     drawRightArrow("Red");
      rightArrowShown= 'F'; 
    }
  }
  if(carsRight[1] == 0){
    if(rightArrowShown != 'S'){
    drawRightArrow("Yellow");
    rightArrowShown = 'S'; 
    }
  }
  if(carsRight[0] == 1 & carsRight[1] == 1){
    if(rightArrowShown != 'B'){ 
    drawRightArrow("Black");
      rightArrowShown = 'B'; 
    }
  }
  Serial.println("Displaying info!"); 
  
}

void displayInfoOld(){
  int leftMode,rightMode; 
 showLeft();
  if(carsRight[0] == 0) {
    if(isRightSlowShown == true) {
      isImageShown[1] = false; 
    }
     showCarsRight(true, 0);
  }
  if(carsRight[1] == 0){
    isRightSlowShown = true; 
    showCarsRight(true, 1); 
  }
  if(carsRight[0] == 1 & carsRight[1] == 1){
    showCarsRight(false, 0);
  }
  Serial.println("Displaying info!"); 
  
}


bool joinNetwork(){
  String joinCommand = JOIN_NETWORK_COMMAND_PREFIX + String("\"") + NETWORK_SSID + String("\",\"") +PASSWORD + String("\"");
  wifi.println(joinCommand);
  Serial.println("Join Command: " + joinCommand); 
}

bool joinServer(){
  String joinCommand = SUT_UP_TCP_PREFIX + String("\"") +SERVER + String("\",") + String(PORT);
  wifi.println( joinCommand);
  
  Serial.println("Join Server: " + joinCommand); 
 
  
}

bool sendData(String data){
  int len = data.length(); 
  String requestSendCommand = SEND_INFO_PREFIX + String(len); 
  wifi.println(requestSendCommand);
  Serial.println("Request For Send: " + requestSendCommand); 
  delay(500); 
  String dataCommand = data; 
  wifi.flush(); 
  wifi.println(data); 
  Serial.println("Message Data: " + data); 

}

bool determineIfFromServer(){

}

String getNewLaneInformation(){
  
  String command = getServerResponse();
 return command; 

}

bool closeSocket(){

}

void sendBikeID(){
  sendData(ID_STATEMENT); 
}

void sendHeading(){
  int h = getHeading();
  Serial.println("Heading is!!!!!!!!!!!!! " + String(h)); 
  String msg = HEADING_PREFIX + String(h); 
  sendData(msg); 
}

void parseData(String command){
  
  int leftStart = command.indexOf('L') + 1; 
  command = command.substring(leftStart-1);
  leftStart = command.indexOf('L') + 1;
  int leftEnd = command.indexOf('R');
  int rightStart = command.indexOf('R') + 1; 
  if(leftStart >= 0 & rightStart >= 0){
    
    Serial.println("CUT COMMAND: " + command);  
    int commandLength = command.length();   
      carsLeft[0] = (command.substring(leftStart + 1, leftStart + 2)).toInt();
      carsLeft[1] =  (command.substring(leftStart + 3, leftStart + 4)).toInt();
      carsRight[0] = (command.substring(rightStart + 1, rightStart + 2)).toInt();
      carsRight[1] =  (command.substring(rightStart + 3, rightStart + 4)).toInt();
      Serial.println("Cars Left: " + String(carsLeft[0]) + "  " + String(carsLeft[1])); 
      Serial.println("Cars Right: " + String(carsRight[0]) + "  " + String(carsRight[1])); 
  }
  else{
    Serial.println("Bogus command: " + command); 
  }
}
void drawLeftArrowOld(String color){
  uint16_t colorToShow; 
  if(color == "Red") colorToShow = ST7735_RED; 
  else if(color == "Green") colorToShow = ST7735_GREEN; 
  else if(color == "Black") colorToShow = ST7735_BLACK;  
  else if(color == "Yellow") colorToShow = ST7735_YELLOW; 
  
  
 
     tft.setRotation(3);
    tft.fillTriangle(20, 68, 60, 48, 60, 88, colorToShow);
   tft.setRotation(0); 
  
}

void drawRightArrowOld(String color){
  uint16_t colorToShow; 
  if(color == "Red") colorToShow = ST7735_RED; 
  else if(color == "Green") colorToShow = ST7735_GREEN; 
  else if(color == "Black") colorToShow = ST7735_BLACK;  
  else if(color == "Yellow") colorToShow = ST7735_YELLOW; 
  
     
 
 
 tft.setRotation(3);
    tft.fillTriangle(140, 68, 100, 48, 100, 88, colorToShow);
   tft.setRotation(0); 
}

void displayHeading(int heading){
  String hello = "Heading: " + String(heading); 
  char array[hello.length() + 1]; 
  hello.toCharArray(array, hello.length()); 
  tft.setRotation(3); 
  tft.fillRect(headingLoc[0],headingLoc[1], 160, 10, ST7735_BLACK); 
  tft.setCursor(headingLoc[0], headingLoc[1]); 
  tft.setTextColor(ST7735_GREEN); 
  tft.setTextWrap(true); 
  tft.setTextSize(1); 
  tft.print(hello); 
  tft.setRotation(0); 
}

void displayChangeInHeading(){
  tft.setRotation(3); 
  tft.fillRect(headingLoc[0],headingLoc[1], 160, 10, ST7735_BLACK); 
  tft.setCursor(headingLoc[0], headingLoc[1]); 
  tft.setTextColor(ST7735_GREEN); 
  tft.setTextSize(1); 
  tft.setTextWrap(true); 
  tft.print("Sending New Heading"); 
  tft.setRotation(0); 
}

void displayInitStatus(bool initStatus){
  String toDisplay = "Init Status: "; 
  if(initStatus){
    toDisplay += "Initialized";
  }
  else{
    toDisplay += "Initializing"; 
  }
  tft.setRotation(3); 
  tft.fillRect(initLoc[0],initLoc[1], 160, 10, ST7735_BLACK); 
  tft.setCursor(initLoc[0], initLoc[1]); 
  tft.setTextColor(ST7735_GREEN); 
  tft.setTextSize(1); 
  tft.setTextWrap(true); 
  tft.print(toDisplay); 
  tft.setRotation(0);
}

void hideInitStatus(){
  tft.setRotation(3); 
  tft.fillRect(initLoc[0],initLoc[1], 160, 10, ST7735_BLACK); 
  tft.setRotation(0);
}

void displayIntersection(String intersection){
  tft.setRotation(3); 
  tft.fillRect(intersectionLoc[0],intersectionLoc[1], 160, 10, ST7735_BLACK); 
  tft.setCursor(intersectionLoc[0], intersectionLoc[1]); 
  tft.setTextColor(ST7735_GREEN); 
  tft.setTextSize(1); 
  tft.setTextWrap(true); 
  tft.print("Approaching:" + intersection); 
  tft.setRotation(0);
}

String getIntersection(){
  
  if(SHOULD_ASK_FOR_INTERSECTION_NAME){
    sendData(INTERSECTION_REQUEST); 
    delay(1000); 
    String command = getServerResponse();
    //command += getServerResponse(); 
    Serial.println("Response on intersection not parsed: " + command); 
    int start = command.indexOf("I,"); 
    String intersection = command.substring(start+2); 
    return "Ravenridge & \n Falls of Nuese"; 
  } 
}

void clearIntersection(){
  tft.setRotation(3);
  tft.fillRect(0,20,160, 120,ST7735_BLACK); 
  tft.setRotation(0); 
}

void displayGraphicIntersection(){
  //displays an intersection....
  tft.setRotation(3);  
  //cross bars 
  tft.drawLine(70,20, 50, 108,ST7735_WHITE); 
  tft.drawLine(90,20,110,108, ST7735_WHITE); 
  tft.drawLine(20, 38, 140, 38,  ST7735_WHITE); 
  tft.drawLine(0, 68, 160, 68,  ST7735_WHITE); 

  //dashes 
   tft.drawLine(80,20, 80, 25,ST7735_YELLOW);
   tft.drawLine(80,32, 80, 38,ST7735_YELLOW);
   //tft.drawLine(80,58, 80, 65,ST7735_YELLOW);
   tft.drawLine(80,73, 80, 83,ST7735_YELLOW);
   tft.drawLine(80,93, 80, 108,ST7735_YELLOW);

   //horizontal 
   tft.drawLine(10,50, 20, 50,ST7735_YELLOW);
   tft.drawLine(30,50, 40, 50,ST7735_YELLOW);
   tft.drawLine(50,50, 60, 50,ST7735_YELLOW);
   
   tft.drawLine(150,50, 140, 50,ST7735_YELLOW);
   tft.drawLine(130,50, 120, 50,ST7735_YELLOW);
   tft.drawLine(110,50, 100, 50,ST7735_YELLOW);
  tft.setRotation(0); 
  
}

void drawLeftArrow(String color){
  uint16_t colorToShow; 
  if(color == "Red") colorToShow = ST7735_RED; 
  else if(color == "Green") colorToShow = ST7735_GREEN; 
  else if(color == "Black") colorToShow = ST7735_BLACK;  
  else if(color == "Yellow") colorToShow = ST7735_YELLOW; 
  
  
 
     tft.setRotation(3);
    tft.fillTriangle(25, 50, 58, 38, 50, 68, colorToShow);
   tft.setRotation(0); 
  
}

void drawRightArrow(String color){
  uint16_t colorToShow; 
  if(color == "Red") colorToShow = ST7735_RED; 
  else if(color == "Green") colorToShow = ST7735_GREEN; 
  else if(color == "Black") colorToShow = ST7735_BLACK;  
  else if(color == "Yellow") colorToShow = ST7735_YELLOW; 
  
     
 
 
 tft.setRotation(3);
    tft.fillTriangle(135, 50, 102, 38, 110, 68, colorToShow);
   tft.setRotation(0); 
}

void displayGraphicIntersectionOld(){
  //displays an intersection....
  tft.setRotation(3);  
  tft.drawLine(60,20, 60, 108,ST7735_WHITE); 
  tft.drawLine(100,20,100,108, ST7735_WHITE); 
  tft.drawLine(20, 48, 140, 48,  ST7735_WHITE); 
  tft.drawLine(20, 70, 140, 70,  ST7735_WHITE); 
  tft.setRotation(0); 
  
}
void displaySearching(){
  tft.setRotation(3); 
  
  tft.setCursor(search[0], search[1]); 
  tft.setTextColor(ST7735_BLUE); 
  tft.setTextWrap(true); 
  tft.setTextSize(1); 
  tft.print("Searching For Intersection"); 
  tft.setRotation(0);
  
}

void hideSearch(){
 tft.setRotation(3); 
  
 tft.fillRect(search[0], search[1], 160,10, ST7735_BLACK); 
  tft.setRotation(0);
  
}

void displayCardinal(){
  int i = getHeading(); 
  
  String cardinal = determineCardinalDirection(i); 
  if(lastCardinal == cardinal) return; 
  else{
  tft.setRotation(3); 
  tft.fillRect(card[0], card[1], 80,40, ST7735_BLACK); 
  if(cardinal == "N" | cardinal == "S" | cardinal == "W" | cardinal == "E"){
     tft.setCursor(card[0] + 10, card[1]); 
  }
  else{
  tft.setCursor(card[0], card[1]); 
  }
  tft.setTextColor(ST7735_WHITE); 
  tft.setTextWrap(true); 
  tft.setTextSize(5); 
  tft.print(cardinal); 
  tft.setRotation(0);
  lastCardinal = cardinal; 
  }
}

String determineCardinalDirection(int heading){
  String c; 
  if(heading >0 & heading < 22) c = "N"; 
  if(heading >22 & heading < 67) c = "NE"; 
  if(heading >67 & heading < 112) c = "E"; 
  if(heading >112 & heading < 157) c = "SE"; 
  if(heading >157 & heading < 202) c = "S"; 
  if(heading >202 & heading < 247) c = "SW"; 
  if(heading >247 & heading < 292) c = "W"; 
  if(heading >292 & heading < 337) c = "NW"; 
  if(heading >337 & heading < 360) c = "N"; 
  return c; 
}

void loop() {
  if(isInit){
    
    if(digitalRead(BUTTON)){
      while(digitalRead(BUTTON));
      while(!digitalRead(BUTTON));
      long timeStart = millis(); 
      while(digitalRead(BUTTON));
      long timeEnd = millis(); 
      long elapsed = timeEnd - timeStart; 
      if(elapsed < 200){
        demoMode = 0; 
      }
      else if(elapsed < 800){
        demoMode = 1; 
      }
      else demoMode = 2;
      Serial.println("SENDING NEW HEADING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"); 
      displayChangeInHeading(); 
      delay(5000); 
      sendHeading(); 
      delay(2000);
    }
    
   //clearIntersection(); 
    //displayGraphicIntersection();
    displayIntersection(intersection);
    sendInfoRequest(); 
    //displayInfo(); 
    String  newInfo = getNewLaneInformation();
    Serial.println("MESSAGE FOR CAR LOC: " + newInfo);
   //displayInfo(); 
    parseData(newInfo);
    displayGraphicIntersection(); 
    displayInfo();
    if(!hasInitCleared){
    hideInitStatus(); 
hasInitCleared = true; 
    }
    int p = getHeading(); 
  }
  // put your main code here, to run repeatedly:

}

