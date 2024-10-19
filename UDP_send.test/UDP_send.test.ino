/*
 UDPSendReceiveString:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender

 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.

 created 21 Aug 2010
 by Michael Margolis

 This code is in the public domain.
 */


#include <PortentaEthernet.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// The IP address will be dependent on your local network:
IPAddress ip(192, 168, 50, 4);

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {

  // start the Ethernet
  Ethernet.begin(ip);

  // Open serial communications:
  Serial.begin(57600);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);

  pinMode(BTN_USER, INPUT);
  pinMode(LED_D0, OUTPUT);
  pinMode(D0, OUTPUT);

}

void loop() {
  bool buttonState = !digitalRead(BTN_USER);
  Serial.println(buttonState);
  if(buttonState) {
    Udp.beginPacket({192, 168, 50, 3}, 8888);
    Udp.endPacket();
    digitalWrite(LED_D0, true);
    digitalWrite(D0, true);

  } else {
    digitalWrite(LED_D0, false);
    digitalWrite(D0, false);

  }
  delay(10);
}










//