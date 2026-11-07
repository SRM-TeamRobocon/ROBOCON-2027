

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x00, 0x04};

IPAddress client_ip(192, 168, 1, 50);
IPAddress agent_ip(192, 168, 1, 100);
uint16_t agent_port = 8888;

void setup()
{
  Serial.begin(115200);

  Serial.println("START");

  Ethernet.begin(mac, client_ip);
  delay(2000);

  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  Serial.println("ETHERNET DONE");
}

void loop()
{
  Serial.println("ALIVE");
  delay(1000);
}