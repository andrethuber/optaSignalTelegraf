#define LENOF(array) (sizeof(array) / sizeof(array[0]))

#define MAX_PING 100                // ms, Maximum permisseble delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000         // ms, Time delay between sendig heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 2000  // ms, Time allowed without reciving a heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100        // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 200         // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time of relays
#define HEARTBEAT_LED LED_RESET     // pinId, The led used for indicating a local heartbeat

#define PORT 8888  // Port that the controllers send and listen on.

#include <Ethernet.h>
#include <EthernetUdp.h>


/* Documentaion

  Signal - The electrical signal that we are simulating.
  Packet - The udp packet that travels between stations.
  Controller - The Arduino Opta plc that runs this code.
  Act Lock - A status that if set:
    Prevents ringing the bells.
      (Prevents closing and opening of 'telegraphSigOut relay.)
    Prevents sending telegraph packets.
    Prevents sending heartbeat. -- To be reconsidred --


  't' = telegraph signal
  'b' = no send because bouncing detected
  'i' = no send because 'inputLock' is true (usualy when reciving a signal)
  'e' = general error
  'a' = "acknowledged", response to received 't' packet
  'h' = heartbeat, gets sent every ~1 sec
  'l' = set 'actLock'
  'r' = relesse 'actLock'

  Q1 - 'telegraphSigOut'
  Q2 - 'actLock'
  Q3 - '!hasReceivedHeartbeat'

TODO:
  - OTA
  - Prevent sending packets too frequently, insted: delay packet and give a warning to operator (TXP). As opposed to current behavior of seting 'actLock'.
  - Remote error reporting (sms/email)
*/

bool blink = true;

uint8_t dipPins[] = { A2, A3, A4, A5, A6, A7 };  // Pins used to configure IP address (like a dip switch)

// Default IP adresses for the pair (4th octet gets overwritten in 'setup'):
IPAddress ipLocal(192, 168, 50, 2);
IPAddress ipRemote(192, 168, 50, 3);


bool hasReceivedHeartbeat = false;  // Prevents the system from doing most tasks before getting a first heartbeat
bool actLock = false;               // Prevents the system from doing most tasks (ringing the bells (telegraphSigOut), sending telegraph packets)

bool lastTelegraphSigIn;
bool inputLock;

unsigned long lastTelegraphPacket;
unsigned long lastAcknowledgement;

unsigned long lastTelegraphSigOut;

unsigned long lastHeartBeatReceived;
unsigned long lastHeartBeatSent;



EthernetUDP udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // Buffer to hold incoming packet.


void setup() {
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, HIGH);  // Enable HEARTBEAT_LED led at start

  pinMode(LED_D2, OUTPUT);
  digitalWrite(LED_D2, !hasReceivedHeartbeat);
  pinMode(D2, OUTPUT);
  digitalWrite(D2, !hasReceivedHeartbeat);

  Serial.begin(9600);
  //while (!Serial)  // Wait for serial, remove before prod
  //  ;
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


  // Telegraph in:
  pinMode(A0, INPUT);
  pinMode(BTN_USER, INPUT);

  // 'telegraphSigOut':
  pinMode(LED_D0, OUTPUT);
  pinMode(D0, OUTPUT);


  Serial.println("t5");

  // Ethernet/Udp:
  Ethernet.begin(ipLocal);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) Serial.println("Ethernet hardware not found");
  if (Ethernet.linkStatus() == LinkOFF) Serial.println("Ethernet cable is not connected");

  Serial.println("t6");

  udp.begin(PORT);

  digitalWrite(HEARTBEAT_LED, LOW);
  Serial.println("t2");
  delay(50);
  digitalWrite(HEARTBEAT_LED, HIGH);
}

void loop() {

  // Process receiving a telegraph signal:
  if (!actLock && hasReceivedHeartbeat) {  // Do not process receiving a telegraph signal if 'actLock' is true or has not yet received its first heartbeat.
    bool telegraphSigIn = digitalRead(A0) | !digitalRead(BTN_USER);
    if (telegraphSigIn > lastTelegraphSigIn) {
      Serial.println(sendTelegraphPacket());
    }
    lastTelegraphSigIn = telegraphSigIn;  // To detect a rising edge
  }


  // What to do when receive a udp packet:
  uint8_t packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

    Serial.print("Received packet: ");
    Serial.println(packetBuffer);
    if (packetSize > 1) systemError("FATAL: Received a packet with a size greater then 1!");

    if (packetBuffer[0] == 'h') {  // Process heartbeat
      lastHeartBeatReceived = millis();
      hasReceivedHeartbeat = true;
    }
    if (packetBuffer[0] == 'e' && !actLock) systemError("ERROR: Paired station has experienced a error!");  // Process error pakcet exept if 'actLock' is on, to prevent the packets beeing sent back and fourth.
    if (packetBuffer[0] == 'l') systemError("Manually triggred actLock");                                   // For manually triggering act lock
    if (packetBuffer[0] == 'r') actLock = false;                                                            // For manually releasing act lock


    if (!actLock && hasReceivedHeartbeat) {                                    // Disable sending packets if actLock is true, or if has not yet received its first heartbeat.
      if (packetBuffer[0] == 't') Serial.println(onReceiveTelegraphPacket());  // Process telegraph packet
      if (packetBuffer[0] == 'a') lastAcknowledgement = millis();              // Process acknowledgement packet
    }
  }  // end 'if (pakcetSize)'

  if (millis() - lastHeartBeatSent > HEARTBEAT_RATE && !actLock) {  // Sends heartbeats, exept if 'actLock'
    lastHeartBeatSent = millis();
    udpSend('h');
    blink = !blink;
  }

  if (!actLock && hasReceivedHeartbeat) {                     // Disable acting if actLock is true, or if has not yet received its first heartbeat.
    if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME) {  // Resets 'telegraphSigOut' relay
      digitalWrite(LED_D0, LOW);
      digitalWrite(D0, LOW);
    }
    if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME + INPUTLOCK_DELAY) inputLock = false;  // Resets 'inputLock'

    if (millis() - lastHeartBeatReceived > MAX_TIME_NO_HEARTBEAT) systemError("ERROR: Heartbeat lost!");                                    // Errors if does not receive any heatbeats in enough time
    if (millis() - lastTelegraphPacket > MAX_PING && lastTelegraphPacket > lastAcknowledgement) systemError("ERROR: No acknowledgement!");  // Errors if does not receive an acknowledgement in time
  }


  digitalWrite(HEARTBEAT_LED, blink);
  digitalWrite(LED_D1, actLock);
  digitalWrite(D1, actLock);
  digitalWrite(LED_D2, !hasReceivedHeartbeat);
  digitalWrite(D2, !hasReceivedHeartbeat);
}

uint8_t getIpsFromDip(uint8_t value, bool isLocal) {
  if (isLocal) return value + 100;
  else if (value % 2 == 0) return value + 101;  // if even
  else return value + 99;                       // if odd
}

void udpSend(char code) {
  udp.beginPacket(ipRemote, PORT);
  udp.write(code);
  udp.endPacket();
  Serial.print("Sent packet: ");
  Serial.println(code);
}

char sendTelegraphPacket() {
  if (millis() - lastTelegraphPacket < 50) return ('b');
  if (inputLock) return ('i');
  udpSend('t');
  lastTelegraphPacket = millis();
  return ('t');
}

char onReceiveTelegraphPacket() {
  if (inputLock == true) {
    systemError("ERROR: Received two packets in too short of a time period!");
    return ('e');
  }
  inputLock = true;  // To make sure we dont process the output signal as a input signal.
  digitalWrite(LED_D0, HIGH);
  digitalWrite(D0, HIGH);
  udpSend('a');
  lastTelegraphSigOut = millis();
  return ('a');
}

void systemError(char errorMessage[]) {
  actLock = true;
  Serial.println(errorMessage);
  udpSend('e');  // Tell the paired station that we have experienced a error
  Serial.println("'actLock' has been enabled");
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
