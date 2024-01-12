#include <Arduino.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include <math.h>

#include <ESP32Servo.h>

//Settings
#define SLOW_BOOT 0
#define HOSTNAME "Golf Teeing Robot"
#define FORCE_USE_HOTSPOT 0

void lerpMove(double x, double y, double z, double AA); //lerped movement using kinematics() to make the end effector move gradually along a straight line
void kinematics(double x, double y, double z, double AA); //move arm to coord
double radianToDegree(double); //converts radians to degrees for display purposes
double degreeToRadian(double); //converts degrees to radians, primarily for ease of input when debugging

void PWMControl(double base, double shoulder, double elbow, double wrist);
void closeGrip();
void openGrip();
double servo485Corrector(double angle);
double servo755Corrector(double angle);

void connectWifi();
void setUpUI();
void enterWifiDetailsCallback(Control *sender, int type);
void textCallback(Control *sender, int type);
void lowTeeCallback(Control *sender, int type);
void mediumTeeCallback(Control *sender, int type);
void highTeeCallback(Control *sender, int type);

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripServo;

int basePos = 0;
int shoulderPos = 0;
int elbowPos = 0;
int wristPos = 0;
int gripPos = 0;

int basePin = 14;
int shoulderPin = 27;
int elbowPin = 26;
int wristPin = 25;
int gripPin = 33;

//for keeping track of current end effector position to allow for it to "lerp" (to move gradually to the target along a straight line rather than however the arm gets there fastest)
double xPos = 0;
double yPos = 0;
double zPos = 0;

//UI handles
uint16_t wifi_ssid_text, wifi_pass_text;

//arm descriptor constants for function Kinematics()
    const double zOff = 6.7; //"Z Offset" - vertical distance from base to axis of shoulder joint
    const double linkC = 14.6; //"link C" - length of "link C" running between axes of "shoulder" and "elbow"
    const double linkB = 27.9; //"link B" - length of "link B" running between axes of "elbow" and "wrist"
    const double linkA = 8.7; //"link A" - length of "link A" running between axis of "wrist" and tip of end effector

// This is the main function which builds our GUI
void setUpUI() {
	//Turn off verbose debugging
	ESPUI.setVerbosity(Verbosity::Quiet);

	/*
	 * Tab: Basic Controls
	 * This tab contains all the basic ESPUI controls, and shows how to read and update them at runtime.
	 *-----------------------------------------------------------------------------------------------------------*/
  auto grouptab = ESPUI.addControl(Tab, "", "Golf Tee Height");
	//The parent of this button is a tab, so it will create a new panel with one control.
	auto groupbutton = ESPUI.addControl(Button, "Golf Tee Height", "Low", Dark, grouptab, lowTeeCallback);
	//However the parent of this button is another control, so therefore no new panel is
	//created and the button is added to the existing panel.
	ESPUI.addControl(Button, "", "Medium", Alizarin, groupbutton, mediumTeeCallback);
	ESPUI.addControl(Button, "", "High", Alizarin, groupbutton, highTeeCallback);

  /*
	 * Tab: WiFi Credentials
	 * You use this tab to enter the SSID and password of a wifi network to autoconnect to.
	 *-----------------------------------------------------------------------------------------------------------*/
	auto wifitab = ESPUI.addControl(Tab, "", "WiFi Credentials");
	wifi_ssid_text = ESPUI.addControl(Text, "SSID", "", Alizarin, wifitab, textCallback);
	//Note that adding a "Max" control to a text control sets the max length
	ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
	wifi_pass_text = ESPUI.addControl(Text, "Password", "", Alizarin, wifitab, textCallback);
	ESPUI.addControl(Max, "", "64", None, wifi_pass_text);
	ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab, enterWifiDetailsCallback);

	//Finally, start up the UI. 
	//This should only be called once we are connected to WiFi.
	ESPUI.begin(HOSTNAME);

}

void lowTeeCallback(Control *sender, int type) {
  if(type == B_UP) {
    Serial.println("LowCallback");
    lerpMove(30, 0, 10, 90);
    delay(400);
    lerpMove(30, 0, 0, 90);
    delay(400);
    closeGrip();
    lerpMove(30, 20, 3, 0);
    delay(400);
    openGrip();
    lerpMove(20, 10, 30, 0);
  }
}

void mediumTeeCallback(Control *sender, int type) {
  if(type == B_UP) {
    Serial.println("MediumCallback");
    lerpMove(30, 0, 10, 90);
    delay(400);
    lerpMove(30, 0, 0, 90);
    delay(400);
    closeGrip();
    lerpMove(30, 29, 5.5, 0);
    delay(400);
    openGrip();
    lerpMove(20, 10, 30, 0);
  }
}

void highTeeCallback(Control *sender, int type) {
  if(type == B_UP) {
    Serial.println("HighCallback");
    lerpMove(30, 0, 10, 90);
    delay(400);
    lerpMove(30, 0, 0, 90);
    delay(400);
    closeGrip();
    lerpMove(30, 38, 8, 0);
    delay(400);
    openGrip();
    lerpMove(20, 10, 30, 0);
  }
}

void setup() {
	Serial.begin(115200); //Starts serial transmission for debugging purposes
	//while(!Serial);
	if(SLOW_BOOT) delay(5000); //Delay booting to give time to connect a serial monitor
	connectWifi();
	WiFi.setSleep(false); //For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
	setUpUI();

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	baseServo.setPeriodHertz(50);
	baseServo.attach(basePin, 553, 2425);
  shoulderServo.setPeriodHertz(50);
  shoulderServo.attach(shoulderPin, 556, 2410);
  elbowServo.setPeriodHertz(50);
  elbowServo.attach(elbowPin, 556, 2410);
  wristServo.setPeriodHertz(50);
  wristServo.attach(wristPin, 553, 2425);
  gripServo.setPeriodHertz(50);
	gripServo.attach(gripPin, 500, 2500);
  
  kinematics(0, 20, 20, 0); //starting position
  xPos = 0;
  yPos = 20;
  zPos = 20;
}

void loop() {

}

void readStringFromEEPROM(String& buf, int baseaddress, int size) {
	buf.reserve(size);
	for (int i = baseaddress; i < baseaddress+size; i++) {
		char c = EEPROM.read(i);
		buf += c;
		if(!c) break;
	}	
}

void connectWifi() {
	int connect_timeout;
  WiFi.setHostname(HOSTNAME);
  Serial.println("Begin wifi...");

	//Load credentials from EEPROM 
	if(!(FORCE_USE_HOTSPOT)) {
		yield();
		EEPROM.begin(100);
		String stored_ssid, stored_pass;
		readStringFromEEPROM(stored_ssid, 0, 32);
		readStringFromEEPROM(stored_pass, 32, 96);
		EEPROM.end();
	
		//Try to connect with stored credentials, fire up an access point if they don't work.
		WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
		
		connect_timeout = 28; //7 seconds
		while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
			delay(250);
			Serial.print(".");
			connect_timeout--;
		}
	}
	
	if (WiFi.status() == WL_CONNECTED) {
		Serial.println(WiFi.localIP());
		Serial.println("Wifi started");

		if (!MDNS.begin(HOSTNAME)) {
			Serial.println("Error setting up MDNS responder!");
		}
	} else {
		Serial.println("\nCreating access point...");
		WiFi.mode(WIFI_AP);
		WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
		WiFi.softAP(HOSTNAME);

		connect_timeout = 20;
		do {
			delay(250);
			Serial.print(",");
			connect_timeout--;
		} while(connect_timeout);
	}
}

void enterWifiDetailsCallback(Control *sender, int type) {
	if(type == B_UP) {
		Serial.println("Saving credentials to EPROM...");
		Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
		Serial.println(ESPUI.getControl(wifi_pass_text)->value);
		unsigned int i;
		EEPROM.begin(100);
		for(i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++) {
			EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
			if(i==30) break; //Even though we provided a max length, user input should never be trusted
		}
		EEPROM.write(i, '\0');

		for(i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++) {
			EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
			if(i==94) break; //Even though we provided a max length, user input should never be trusted
		}
		EEPROM.write(i + 32, '\0');
		EEPROM.end();
	}
}

void textCallback(Control *sender, int type) {
	//This callback is needed to handle the changed values, even though it doesn't do anything itself.
}


//Function to work out joint angles from 
void kinematics(double x, double y, double z, double AA){
  //user input variables for debugging the function, commented out for regular use
  /*double x = 20; //target coord set by user, treating base of base rotator as origin
  double y = 20;
  double z = 20;
  double AA = degreeToRadian(0);*/ //"Approach Angle" user set approach angle for effector, relative to XY-plane
  //all angle operation in C++ use radians: 1.5708 is 90 degrees for testing purposes

    
  //kinematics variables
  double RZO; //"Relative Z Offset" - models changes in target Z coord by spoofing origin position relative to it
  double APD; //"Angled Plane Distance" distance from effector to base
  double AAO; //"Approach Angle Offset" is used to correct approach angle from being relative to a line between effector and shoulder, to being between effector and base
  double theta; //corrected approach angle relative to a line between effector and shoulder - uses AAO
  double r1; //internal distance from effector to shoulder
  double r2; //internal distance from wrist to shoulder
  double angleB1; //three variables for internal angle between "link C" and z-axis about "shoulder" joint
  double angleB2;
  double angleB3;
  double angleC1; //two variables for internal angle between "link A" and "link B" about "wrist" joint
  double angleC2;

  //output variables
  double wristAngle; //internal angle of "wrist" joint between "link A" and "link B" - joint attached to "elbow" via link b
  double elbowAngle; //internal angle of "elbow" joint between "link B and "link C" - joint attached to "shoulder" via link c
  double shoulderAngle; //internal angle of "shoulder" joint between "link C" and z-axis - first joint directly attached to base rotator
  double baseAngle; //horizontal angle of base rotator relative to y-axis

  RZO = zOff - z;
  baseAngle = atan(y/x);
  APD = sqrt(pow(x, 2) + pow(y, 2));
  r1 = sqrt(pow(APD, 2) + pow(RZO, 2));

  AAO = atan(RZO/APD);
  theta = degreeToRadian(AA) - AAO;

  r2 = sqrt((pow(linkA, 2) + pow(r1, 2)) - (2*linkA*r1*cos(theta)));

  elbowAngle = acos((pow(r2, 2) - (pow(linkB, 2) + pow(linkC, 2))) / (-2 * linkB * linkC));

  angleB1 = acos((pow(linkB, 2) - (pow(r2, 2) + pow(linkC, 2))) / (-2 * r2 * linkC));
  angleC1 = degreeToRadian(180) - elbowAngle - angleB1;

  angleB2 = asin((linkA * sin(theta)) / r2);
  angleB3 = degreeToRadian(90) - AAO;
  shoulderAngle = angleB1 + angleB2 + angleB3;



  angleC2 = degreeToRadian(450) - theta - AAO - angleC1 - elbowAngle - angleB1 - angleB2 - angleB3;
  /* angleC2 would be 540 minus all other internal values because the geometry of the arm excluding the base rotator can
  be expressed as a pentagon but the angle between AAO (the height of the "shoulder" axis above the base) and the base
  meet at a right angle, so the 90 degrees has already been subtracted leaving only 450 degrees to account for. */
  
  wristAngle = angleC1 + angleC2;

  Serial.print("Wrist angle: ");
  Serial.println(radianToDegree(wristAngle));
  Serial.print("Elbow angle: ");
  Serial.println(radianToDegree(elbowAngle));
  Serial.print("Shoulder angle: ");
  Serial.println(radianToDegree(shoulderAngle));
  Serial.print("Base angle: ");
  Serial.println(radianToDegree(baseAngle));

  PWMControl(radianToDegree(baseAngle), radianToDegree(shoulderAngle), radianToDegree(elbowAngle), radianToDegree(wristAngle));
}

void lerpMove(double x, double y, double z, double AA){
  double xDiff;
  double yDiff;
  double zDiff;
  int lerpPrecision;
  int iterations;

  Serial.print("Lerping Initiated using AA: ");
  Serial.println(AA);
  lerpPrecision = 100;
  iterations = 0;
  xDiff = x - xPos;
  yDiff = y - yPos;
  zDiff = z - zPos;

  while(iterations < lerpPrecision){
    xPos = xPos + (xDiff / lerpPrecision);
    yPos = yPos + (yDiff / lerpPrecision);
    zPos = zPos + (zDiff / lerpPrecision);
    Serial.print("AA: ");
    Serial.println(AA);
    Serial.println("");
    kinematics(xPos, yPos, zPos, AA);
    iterations += 1;
  }
}

void PWMControl(double base, double shoulder, double elbow, double wrist){
  baseServo.write(servo485Corrector(base+20)); 
  shoulderServo.write(servo755Corrector(shoulder-60));
  elbowServo.write(servo755Corrector(195-elbow));
  wristServo.write(servo485Corrector(wrist-85));
}

void openGrip(){
  //set for empty stock gripper, will need rework for modded gripper with load
  gripServo.write(servo422Corrector(20));
}

void closeGrip(){
  //set for empty stock gripper, will need rework for modded gripper with load
  gripServo.write(servo422Corrector(105));
}

double radianToDegree (double inputNum){
    const double pi = 4*atan(1);
    double outNum = 0;

    outNum = inputNum * (180 / pi);
    return outNum;
}

double degreeToRadian (double inputNum){
    const double pi = 4*atan(1);
    double outNum = 0;

    outNum = inputNum / (180 / pi);
    return outNum;
}

double servo485Corrector(double angle){
  double outAngle;
  outAngle = (angle / 190) * 180;
  return outAngle;
}

double servo755Corrector(double angle){
  double outAngle;
  outAngle = (angle / 202) * 180;
  return outAngle;
}

double servo422Corrector(double angle){
  double outAngle;
  outAngle = (angle / 195) * 180;
  return outAngle;
}