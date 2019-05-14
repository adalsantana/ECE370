#include <WiFi101.h>
#include <WiFiUdp.h>
#include "secrets.h"

#define MOTOR1_PINA 10
#define MOTOR1_PINB 11
#define MOTOR2_PINA 9 
#define MOTOR2_PINB 5 

char ssid[] = SECRET_SSID;  // network SSID)
char pass[] = SECRET_PASS;  //network password

int status = WL_IDLE_STATUS; 
WiFiServer server(80); 

WiFiUDP Udp; 

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

void setup() {
  Serial.begin(9600);
  motorsetup();
  APsetup(); 
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

void loop() {
    //WiFiClient client = server.available();   // listen for incoming clients
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
      //int len = Udp.read(packetBuffer, 255);
      Serial.println("Contents:");
      //Serial.println(packetBuffer); 
      Serial.println(udp.velocity);
      Serial.println(udp.theta);
      Serial.println(udp.rst); 
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
