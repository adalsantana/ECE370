#define SECRET_SSID "feather"
#define SECRET_PASS "pass"
#define wifi_channel  1
#define UDP_PORT_SEND 4242
#define UDP_PORT_LISTEN 2363
#define returnRate 0.1 //Hz
#define DESTINATION_OCT_1 192
#define DESTINATION_OCT_2 168
#define DESTINATION_OCT_3 1
#define DESTINATION_OCT_4 100
typedef struct udp_recv{
  double velocity; 
  double theta; 
  int rst; 
}__attribute__((packed)) udp_recv_t; 

typedef struct test{
  double testval;   
}__attribute__((packed))test_t; 


typedef struct udp_send{
  double imu[6]; 
  double odo[3]; 
  double heading; 
}__attribute__((packed)) udp_send_t; 
