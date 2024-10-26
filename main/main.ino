#define LENOF(array) (sizeof(array) / sizeof(array[0]))

#define MAX_PING 100                // ms, Maximum permissable delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000         // ms, Time delay between sending heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 5000  // ms, Time allowed without receiving heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100        // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 20          // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time off relays
#define BLINK_LED LED_RESET         // pinId, The led used for indicating a local heartbeat
#define BOUNCE_TIME 20              // ms, maximum possible bounce time for inputs

#define PORT 8888  // Port that the controllers send and listen on.

#include "library.c"

#include <Ethernet.h>
#include <EthernetUdp.h>


/* Documentaion

  Signal - The electrical signal that we are simulating.
  Packet - The udp packet that travels between stations.
  Controller - The Arduino Opta plc that runs this code.

  't' = Telegraph packet
  'a' = "acknowledged", response to received 't' packet
  'b' = Aborted sending packet because bouncing detected
  'i' = Aborted sending packet because 'inputLock' is true (usually when receiving a signal)
  'e' = General error
  'h' = heartbeat, gets sent every ~1 sec

  Q1 - 'telegraphSigOut'
  Q2 - 'warningLamp'

TODO:
  - OTA
  - Prevent sending packets too frequently, insted: delay packet and give a warning to operator (TXP). As opposed to current behavior of seting 'actLock'.
  - Logging
  - Remote error reporting (sms/email)
  - Handle 'millis' overflow
*/

bool blink = true;

uint8_t dipPins[] = { A2, A3, A4, A5, A6, A7 };  // Pins used to configure IP address (like a dip switch)

// Default IP adresses for the pair (4th octet gets overwritten in 'setup'):
IPAddress ipLocal(192, 168, 50, 2);
IPAddress ipRemote(192, 168, 50, 3);


bool warningLamp = true;

bool previousTelegraphSigIn;
bool inputLock;

unsigned long lastTelegraphPacket;
unsigned long lastAcknowledgement;

unsigned long lastTelegraphSigOut;

unsigned long lastHeartBeatReceived;
unsigned long lastHeartBeatSent;



EthernetUDP udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // Buffer to hold incoming packet.


void setup() {
  pinMode(BLINK_LED, OUTPUT);
  digitalWrite(BLINK_LED, HIGH);  // Enable BLINK_LED led at start of setup

  Serial.begin(9600);
  while (!Serial)  // Wait for serial, remove before prod
    ;
  Serial.println("t1");

  initDipPins(dipPins, LENOF(dipPins));

  // Figure out local IP and IP of paired controller:
  uint8_t dipValue = readDipPins(dipPins, LENOF(dipPins));  // Decimal interpretation of dip pins
  ipLocal[3] = getIpsFromDip(dipValue, 1);                  // Get local IP
  ipRemote[3] = getIpsFromDip(dipValue, 0);                 // Get remote IP

  Serial.print("ipLocal = ");
  Serial.println(ipLocal);
  Serial.print("ipRemote = ");
  Serial.println(ipRemote);


  Serial.println("t2");

  // Ethernet/Udp:
  Ethernet.begin(ipLocal);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) Serial.println("Ethernet hardware not found");
  if (Ethernet.linkStatus() == LinkOFF) Serial.println("Ethernet cable is not connected");

  Serial.println("t3");

  udp.begin(PORT);



  // Telegraph signal in:
  pinMode(A0, INPUT);
  pinMode(BTN_USER, INPUT);

  // Error acknowledge button:
  pinMode(A1, INPUT);

  // Telegraph signal out:
  pinMode(LED_D0, OUTPUT);
  pinMode(D0, OUTPUT);

  // Warning lamp:
  pinMode(LED_D1, OUTPUT);
  pinMode(D1, OUTPUT);

  // Blink led is handled at start of setup.

  digitalWrite(LED_D1, warningLamp);  // Updates warning lamp leds
  digitalWrite(D1, warningLamp);

  digitalWrite(BLINK_LED, LOW);  // Rapidly blink BLINK_LED at end of setup
  delay(50);
  digitalWrite(BLINK_LED, HIGH);
  Serial.println("t4");
}

void loop() {

  // Process receiving a telegraph signal (button press):
  bool telegraphSigIn = digitalRead(A0) | !digitalRead(BTN_USER);
  if (telegraphSigIn > previousTelegraphSigIn) {  // If 'telegraphSigIn' has risen
    Serial.println(sendTelegraphPacket());        // Attemps to send a telegraph packet, and prints the response.
  }
  previousTelegraphSigIn = telegraphSigIn;  // To detect a rising edge

  // Process 'errorAckBtn' press:
  if (digitalRead(A1) && warningLamp) onErrorAck();  // Triggers 'onErrorAck' if error ack button is pressed



  // What to do when a udp packet is received:
  uint8_t packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet ");
    Serial.print(packetSize);
    Serial.print(udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE));
    Serial.print(": ");
    Serial.println(packetBuffer);

    if (packetSize > 1) throwError("WARN: Received a packet with a size greater then 1!");  // No packets are supposed to be more than 1 long, so if it is, something is awry.

    if (packetBuffer[0] == 'h') onReceiveHeartbeat();                                         // Process heartbeat
    if (packetBuffer[0] == 'e') throwError("WARN: Paired station has experienced a error!");  // Process error pakcet
    if (packetBuffer[0] == 't') Serial.println(onReceiveTelegraphPacket());                   // Process telegraph packet
    if (packetBuffer[0] == 'a') lastAcknowledgement = millis();                               // Process acknowledgement packet
  }                                                                                           // end 'if (packetSize)'

  if (millis() - lastHeartBeatSent > HEARTBEAT_RATE) {  // Sends heartbeats
    lastHeartBeatSent = millis();
    udpSend('h');
    digitalWrite(BLINK_LED, blink);  // Toggle blink led
    blink = !blink;
  }
  if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME) {  // Resets 'telegraphSigOut' relay
    digitalWrite(LED_D0, LOW);
    digitalWrite(D0, LOW);
  }
  if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME + INPUTLOCK_DELAY) inputLock = false;                                                           // Resets 'inputLock' after 'telegraphSigIn' relay has had time to open.
  if (millis() - lastHeartBeatReceived > MAX_TIME_NO_HEARTBEAT && !warningLamp) throwError("WARN: Heartbeat lost!");                                    // Errors if does not receive any heatbeats in enough time
  if (millis() - lastTelegraphPacket > MAX_PING && lastTelegraphPacket > lastAcknowledgement && !warningLamp) throwError("WARN: No acknowledgement!");  // Errors if does not receive an acknowledgement in time
}


void udpSend(char code) {
  Serial.print("Sent packet ");
  Serial.print(udp.beginPacket(ipRemote, PORT));
  Serial.print(udp.write(code));
  Serial.print(udp.endPacket());
  Serial.print(": ");
  Serial.println(code);
}

char sendTelegraphPacket() {
  if (millis() - lastTelegraphPacket < BOUNCE_TIME) return ('b');
  if (inputLock) return ('i');
  udpSend('t');
  lastTelegraphPacket = millis();
  return ('t');
}

char onReceiveTelegraphPacket() {
  if (inputLock) {
    throwError("WARN: Received two packets in too short of a time period!");
    return ('e');
  }
  inputLock = true;  // To make sure we do not process the output signal as a input signal.
  digitalWrite(LED_D0, HIGH);
  digitalWrite(D0, HIGH);
  udpSend('a');
  lastTelegraphSigOut = millis();
  return ('a');
}

void onReceiveHeartbeat() {
  lastHeartBeatReceived = millis();
}

void throwError(char errorMessage[]) {
  if (warningLamp) return;
  warningLamp = true;
  Serial.println(errorMessage);
  udpSend('e');  // Tell the paired station that we have experienced an error
  Serial.println("There has been a error @");
  Serial.println(millis());
  digitalWrite(LED_D1, warningLamp);
  digitalWrite(D1, warningLamp);
}

void onErrorAck() {
  warningLamp = false;
  Serial.println("Error has been acknowledged @");
  Serial.println(millis());
  digitalWrite(LED_D1, warningLamp);
  digitalWrite(D1, warningLamp);
}

void initDipPins(uint8_t pins[], uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    pinMode(pins[i], INPUT);
  }
}

uint8_t readDipPins(uint8_t pins[], uint8_t count) {
  bool states[count];
  uint8_t value = 0;
  for (uint8_t i = 0; i < count; i++) {
    states[i] = digitalRead(pins[i]);
    value = value + pow(2, count - 1 - i) * states[i];
  }
  return value;
}
