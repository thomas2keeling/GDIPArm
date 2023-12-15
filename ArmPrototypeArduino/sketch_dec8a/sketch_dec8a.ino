//includes webserver.h and its dependencies
#include <HTTP_Method.h>
#include <Uri.h>
#include <WebServer.h>

//includes wifi.h and its dependencies
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

//includes kinematics.h and its dependencies
#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 3

//update these for network parameters
const char* ssid = "*****";  // Enter your SSID here
const char* password = "*****";  //Enter your Password here
WebServer server(80);  // Object of WebServer(HTTP port, 80 is defult)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Try Connecting to ");
  Serial.println(ssid);

  // Connect to your wi-fi modem
  WiFi.begin(ssid, password);

  // Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());  //Show ESP32 IP on serial

  server.on("/", handle_root);

  server.begin();
  Serial.println("HTTP server started");
  delay(100);



  Kinematics kinematics_object(N);
  MatrixUtils mat_utils;

  kin.add_joint_axis(0, 0,  1,  4, 0,    0);
  kin.add_joint_axis(0, 0,  0,  0, 1,    0);
  kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

  kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                    0, 1,  0, 6,
                                    0, 0, -1, 2,
                                    0, 0,  0, 1);

  float joint_angles[N] = {PI/2.0, 3, PI};
  float transform[4][4];

  kin.forward(joint_angles, (float*)transform);
  mat_utils.print_matrix((float*)transform, 4, 4, "Transform");

  // Output
  // Transform
  // 0.00    1.00    0.00    -5.00
  // 1.00    -0.00   0.00    4.00
  // 0.00    0.00    -1.00   1.69
  // 0.00    0.00    0.00    1.00
}

void loop() {
  // put your main code here, to run repeatedly:
  webApp();
}

void webApp();
void webAppCustom();
void custom(int, int, int);
void low();
void mid();
void high();

void webApp(){ //This function will handle the webapp, and call all following functions
    int choice = 0;

    server.handleClient();

    switch (choice){
        case 0:
            low();
            break;
        case 1:
            mid();
            break;
        case 2:
            high();
            break;
        case 3:
            webAppCustom();
            break;
        default:
            break;
    }
}

void webAppCustom(){ //Webapp function for custom value entry
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y, z);
}

void custom(int x, int y, int z){ //Handles kinematics
    //work out values using kinematics library
    //send pwm signals to move arm to that position
}

void low(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}

void mid(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}

void high(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}

// HTML & CSS contents which display on web server
String HTML = "<!DOCTYPE html>\
<html>\
<body>\
<h1>My First Web Server with ESP32 - Station Mode &#128522;</h1>\
</body>\
</html>";

// Handle root url (/)
void handle_root() {
  server.send(200, "text/html", HTML);
}