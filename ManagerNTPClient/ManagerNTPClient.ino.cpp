# 1 "/var/folders/ff/__xk2dnn5wx5m8lx4g5m5yvm0000gn/T/tmpJD1xQa"
#include <Arduino.h>
# 1 "/Users/nat/Documents/Arduino/ManagerNTPClient/ManagerNTPClient/ManagerNTPClient.ino"
#include <Arduino.h>


#include <CMMC_Manager.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>

Ticker ntpTicker;
bool shouldReloadNTPTime = true;
bool isNTPReloaderLocked = false;

CMMC_Manager manager(0, LED_BUILTIN);
unsigned int localPort = 2390;




IPAddress timeServerIP;
const char* ntpServerName = "ntp.ku.ac.th";

const int NTP_PACKET_SIZE = 48;

byte packetBuffer[ NTP_PACKET_SIZE];


WiFiUDP udp;
void setup();
void loop();
void getNTPTask();
unsigned long sendNTPpacket(IPAddress& address);
#line 29 "/Users/nat/Documents/Arduino/ManagerNTPClient/ManagerNTPClient/ManagerNTPClient.ino"
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  manager.start();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());


  delay(2000);
  ntpTicker.attach(60000, []() {
    if (isNTPReloaderLocked) return;
    Serial.println("================");
    Serial.printf(" . SET FLAG @[%lu] \r\n", millis());
    Serial.println("================");
    shouldReloadNTPTime = true;
  });
}

void loop()
{
  if (shouldReloadNTPTime) {
    getNTPTask();
  }
}

void getNTPTask() {
  shouldReloadNTPTime = false;

  WiFi.hostByName(ntpServerName, timeServerIP);
  int counter = 0;
  while (true) {
    isNTPReloaderLocked = true;
    sendNTPpacket(timeServerIP);

    delay(3000);

    int cb = udp.parsePacket();
    if (!cb) {
      Serial.printf("[%lu] no packet yet\r\n", counter++);
      if (counter > 100) {
        ESP.reset();
      }
    }
    else {
      Serial.print("packet received, length=");
      Serial.println(cb);

      udp.read(packetBuffer, NTP_PACKET_SIZE);




      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);


      unsigned long secsSince1900 = highWord << 16 | lowWord;
      Serial.print("Seconds since Jan 1 1900 = " );
      Serial.println(secsSince1900);


      Serial.print("Unix time = ");

      const unsigned long seventyYears = 2208988800UL;

      unsigned long epoch = secsSince1900 - seventyYears;

      Serial.println(epoch);



      Serial.print("The UTC time is ");
      Serial.print((epoch % 86400L) / 3600);
      Serial.print(':');
      if ( ((epoch % 3600) / 60) < 10 ) {

        Serial.print('0');
      }
      Serial.print((epoch % 3600) / 60);
      Serial.print(':');
      if ( (epoch % 60) < 10 ) {

        Serial.print('0');
      }
      Serial.println(epoch % 60);
      Serial.println("EXIT GET TIME");
      isNTPReloaderLocked = false;
      break;
    }
  }
}

unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");

  memset(packetBuffer, 0, NTP_PACKET_SIZE);


  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;

  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;



  udp.beginPacket(address, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}