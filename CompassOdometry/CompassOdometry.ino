#include <LSM303.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <math.h>
#include <WiFi101.h>
#include "secrets.h"


#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

#define angleTolerance 10 //+- degrees
#define rotate_speed 0.3

#define Kp  2

#define pi 3.14159


LSM303 imu;

char ssid[] = SECRET_SSID;  // network SSID)
char pass[] = SECRET_PASS;  //network password

int status = WL_IDLE_STATUS; 
WiFiServer server(80); 
WiFiUDP Udp; 

void setup() {
  Serial.begin(9600);
  motorsetup();
  imusetup(); 
  APsetup(); 
}

void imusetup(){
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.read();
  //min and max values gotten from calibrate example for lsm303d
  imu.m_min = (LSM303::vector<int16_t>){-2570, -3354, -5081};
  imu.m_max = (LSM303::vector<int16_t>){+2746, +2649, +587};
}

void APsetup(){
  WiFi.setPins(8, 7, 4, 2);
  while(!Serial);
  if(WiFi.status() == WL_NO_SHIELD){
    //stop program
    while(true);
  }

  // by default the local IP address of will be 192.168.1.1
  // you can override it with the following:
  //WiFi.config(IPAddress(10, 0, 0, 30));
  
  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }
  //wait 10 seconds for connection
  delay(10000);
  //start web server on port 80
  printWiFiStatus(); 
  server.begin();
  Serial.print("\nListening for UDP packets on port ");
  Serial.println(UDP_PORT_LISTEN); 
  Serial.print("Result of UDP.begin ");
  Serial.println(Udp.begin(UDP_PORT_LISTEN));
}

void motorsetup(){
  pinMode(MOTOR1_PINA, OUTPUT); 
  pinMode(MOTOR1_PINB, OUTPUT);
  pinMode(MOTOR2_PINA, OUTPUT); 
  pinMode(MOTOR2_PINB, OUTPUT); 
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
  analogWrite(MOTOR2_PINB, 0);
}

int setSpeed(float s){
  if(s > 1.0) s = 1.0; 
  if(s < 0.0) s = 0.0; 
  int out = (int) (s*255.0); 
  return out; 
}

void setAngularVelocity(float velocity){
  float s = velocity; 
  if(s < 0) s = -s; 
  int sp1 = setSpeed(s);  //speed motor 1
  int sp2 = setSpeed(s);  //speed motor 2
  if(velocity < 0){ //turn clockwise  
    analogWrite(MOTOR1_PINB, 0);
    analogWrite(MOTOR1_PINA, sp1);
    analogWrite(MOTOR2_PINB, 0);
    analogWrite(MOTOR2_PINA, sp2);
  }
  else { //turn counterclockwise
    analogWrite(MOTOR1_PINA, 0);
    analogWrite(MOTOR1_PINB, sp1);
    analogWrite(MOTOR2_PINA, 0);
    analogWrite(MOTOR2_PINB, sp2);
  }
}

void motorsOff(){
  analogWrite(MOTOR1_PINA, 0);
  analogWrite(MOTOR1_PINB, 0);
  analogWrite(MOTOR2_PINA, 0);
  analogWrite(MOTOR2_PINB, 0);
}

void moveToAngle(int targetAngle){ //angle should be given in degrees 
  imu.read(); 
  double currentAngle = imu.heading(); 
  double error = currentAngle-targetAngle; //if negative rotate clockwise 
  error = (int) error % 360; // %make error between 0 (inclusive) and 360.0 (exclusive)
  double dist = error > 180.0 ? 360.0 - error: error; //for rollovers
  Serial.print(" Current Angle: ");
  Serial.println(currentAngle);
  Serial.print(" Distance to target ");
  Serial.println(dist);
  while(abs(error) > angleTolerance){
    
    if(error > 0){//clockwise  
      setAngularVelocity(rotate_speed);
    }
    else{ //clockwise
      setAngularVelocity(-rotate_speed);
    }
    error = currentAngle-targetAngle; 
    error = (int) error % 360; // %make error between 0 (inclusive) and 360.0 (exclusive)
    Serial.print("Current Angle: ");
    imu.read();
    
    Serial.print(imu.heading());
    Serial.print(" Error: ");
    Serial.println(error);
    Serial.print(" Distance to target ");
    Serial.println(dist);
  }
  Serial.print(" Final Error: ");
  Serial.println(error);
  motorsOff(); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      udp_recv udp; 
      memset(&udp, 0, sizeof(udp));
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      int len = Udp.read((byte *) &udp, 255);
      Serial.println("Contents:");
      Serial.println(udp.velocity);
      Serial.println(udp.theta);
      Serial.println(udp.rst); 
      moveToAngle(udp.theta);
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
