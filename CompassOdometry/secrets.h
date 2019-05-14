#define SECRET_SSID "feather"
#define SECRET_PASS "pass"
#define UDP_PORT_LISTEN 2363
#define UDP_PORT_SEND 4242
#define returnRate 10 //Hz

typedef struct udp_recv{
  double velocity; 
  double theta; 
  int rst; 
}__attribute__((packed)) udp_recv_t; 

typedef struct udp_send{
  double imu[6]; 
  //double odo[3]; 
  double heading; 
}__attribute__((packed)) udp_send_t; 
