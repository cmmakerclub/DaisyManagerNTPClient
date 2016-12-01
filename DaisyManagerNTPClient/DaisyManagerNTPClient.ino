#include <Arduino.h>
#include <CMMC_Manager.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>

#define __DEVICE__ID 2

/*
 * UDP Header
 */

Ticker ntpTicker;
Ticker counterDownTicker;
bool shouldReloadNTPTime = true;
bool isNTPReloaderLocked = true;
bool shouldUploadDataToCloud = false;

CMMC_Manager manager(0, LED_BUILTIN);
unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "ntp.ku.ac.th";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

#define WAIT_NTP_IN_SECOND 3

/*
* CURRENT
*/

#define LED_vcc   4
#define LED_gnd   5

uint32_t data_sum = 1;
uint32_t data_count = 1;
float data = 0;
uint16_t analogValue;

int current_s;
int remaining_s;

void init_wifi() {
  manager.start();
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  init_wifi();

  pinMode(LED_vcc, OUTPUT);
  pinMode(LED_gnd, OUTPUT);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  counterDownTicker.attach_ms(1000, []() {
    if (isNTPReloaderLocked) return;
    remaining_s--;
    Serial.printf("REMAINING = %d \r\n", remaining_s);
    if(remaining_s <= 0) {
      Serial.printf("SHOULD UPDATE NTP TIME @[%lu] .\r\n", millis());
      shouldReloadNTPTime = true;
      shouldUploadDataToCloud = true;
    }
  });

  delay(2000);
}

void loop()
{
  if (shouldUploadDataToCloud) {
    shouldUploadDataToCloud = false;
    Push_data();
  }

  if (shouldReloadNTPTime) {
    getNTPTask();
  }

  analogValue = analogRead(A0);
  data_sum += analogValue;
  data_count++;
  Serial.print("A0 = ");
  Serial.println(analogValue);
  delay(100);
}

void getNTPTask() {
  Serial.println("LOCK");
  shouldReloadNTPTime = false;
  isNTPReloaderLocked = true;
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);
  int counter = 0;
  while (true) {
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(WAIT_NTP_IN_SECOND*1000);

    int cb = udp.parsePacket();
    if (!cb) {
      Serial.printf("[%lu] no packet yet\r\n", counter++);
      if (counter > 40) {
        ESP.reset();
      }
    }
    else {
        Serial.print("packet received, length=");
        Serial.println(cb);
        // We'v e received a packet, read the data from it
        udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        // combine the four bytes (two words) into a long integer
        // this is NTP time (seconds since Jan 1 1900):
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        // Serial.print("Seconds since Jan 1 1900 = " );
        // Serial.println(secsSince1900);

        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
        unsigned long epoch = secsSince1900 - seventyYears;
        // print Unix time:

        current_s = epoch%60;
        remaining_s = 60 - current_s;

        Serial.printf("EPOCH = %lu \r\n", epoch);
        Serial.printf("SECONDS = %lu \r\n", current_s); // print the second

        // 0
        // 20
        // 40
        remaining_s = getRemainingS(current_s, 20 * (__DEVICE__ID-1));
        Serial.println("UNLOCK");
        isNTPReloaderLocked = false;
        break;
      }
  }
}

int getRemainingS(int current_s, int offset_from_0s) {
  int ret;
  if (current_s > offset_from_0s) {
    ret = 60 - current_s + offset_from_0s;
  }
  else {
    ret = offset_from_0s - current_s;
  }
  ret -= WAIT_NTP_IN_SECOND;
  Serial.printf("REMAINING = %d \r\n", ret);
  return ret;
}
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void Push_data () {
  isNTPReloaderLocked = true;
  data = (float)data_sum / (float)data_count;
  int tmp = data /10.0;
  data = tmp;
  data /= 10.0;
  data_sum = 0;
  data_count = 0;
  Serial.printf("DATA = %s \r\n", String(data).c_str());
  uploadThingsSpeak(data);
  digitalWrite(LED_vcc, HIGH);
  delay(3000);
  digitalWrite(LED_vcc, LOW);
  isNTPReloaderLocked = false;
}

void uploadThingsSpeak(float data) {
  static const String host = "http://api.thingspeak.com";
  static const String apiKey = "EDEKTGVXZDFN3DO8";

  // We now create a URI for the request
  String url = host+ String("/update");
  //  url += streamId;
  //-----------------------------------------------
  url += "?key=";
  url += apiKey;

  url += "&field";
  url += __DEVICE__ID;
  url += "=";
  url += data;
  //---------------------------------------------- -
  HTTPClient http;
  Serial.print("[HTTP] begin...\n");
  http.begin(url); //HTTP
  int httpCode = http.GET();
  if(httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      // file found at server
      if(httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
          Serial.println(payload);
      }
  } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}
