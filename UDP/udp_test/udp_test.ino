#include <WiFi101.h>
#include <WiFiUdp.h>
#include "secrets.h"
#include <LSM303.h>
#include <Wire.h>

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

char ssid[] = SECRET_SSID;  // network SSID)
char pass[] = SECRET_PASS;  //network password

int status = WL_IDLE_STATUS; 
WiFiServer server(80); 
double udpPacketDelay = (1/returnRate)*1000; //convert to milliseconds
WiFiUDP Udp; 

LSM303 imu;

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

double tick = 0; 
double tock = 0;

void setup() {
  Serial.begin(9600);
  motorsetup();
  APsetup(); 
  imusetup();
  tick = millis();
  tock = millis(); 

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

void readUDP(){
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
      Serial.print("Velocity value: ");
      Serial.println(udp.velocity);
      Serial.print("Theta value: ");
      Serial.println(udp.theta);
      Serial.print("Reset value: ");
      Serial.println(udp.rst); 
      //moveToAngle(udp.theta);
  }
}

void sendUDP(){
  Serial.println("Constructing and sending UDP packet");
  udp_send udp; 
  IPAddress target = {  DESTINATION_OCT_1, 
                        DESTINATION_OCT_2, 
                        DESTINATION_OCT_3, 
                        DESTINATION_OCT_4
                      };
                      
  memset(&udp, 0, sizeof(udp));
  imu.read(); 
  udp.imu[0] = imu.a.x;
  udp.imu[1] = imu.a.y;
  udp.imu[2] = imu.a.z;
  udp.imu[3] = imu.m.x;
  udp.imu[4] = imu.m.y;
  udp.imu[5] = imu.m.z;
  udp.odo[0] = 0; 
  udp.odo[1] = 0; 
  udp.odo[2] = 0; 
  udp.heading = imu.heading(); 
  //char* outgoing = (char *) &udp; 
  Udp.beginPacket(target, UDP_PORT_SEND);
  Udp.write((char *) &udp);
  Udp.endPacket(); 
}

void loop() {
    //WiFiClient client = server.available();   // listen for incoming clients
  readUDP(); 
  if(tick - tock > udpPacketDelay){
    sendUDP(); 
    tock = millis(); 
  }
  tick = millis();    
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
