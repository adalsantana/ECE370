#define SECRET_SSID "feather"
#define SECRET_PASS "fuck"
#define wifi_channel  1
#define UDP_PORT_SEND 4242
#define UDP_PORT_LISTEN 2363

typedef struct udp_recv{
  double velocity; 
  double theta; 
  int rst; 
}__attribute__((packed)) udp_recv_t; 

typedef struct test{
  double testval;   
}__attribute__((packed))test_t; 
