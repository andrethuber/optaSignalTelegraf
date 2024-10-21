#define LENOF(array) (sizeof(array) / sizeof(array[0]))

#define MAX_PING 100                // ms, Maximum permisseble delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000         // ms, Time delay between sendig heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 5000  // ms, Time allowed without reciving a heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100        // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 200         // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time of relays
#define BLINK_LED LED_RESET         // pinId, The led used for indicating a local heartbeat
#define BOUNCE_TIME 50              // ms, maximum possible bounce time for inputs

#define PORT 8888  // Port that the controllers send and listen on.

#include <Ethernet.h>
#include <EthernetUdp.h>


/* Documentaion

  Signal - The electrical signal that we are simulating.
  Packet - The udp packet that travels between stations.
  Controller - The Arduino Opta plc that runs this code.
  Act Lock - A status that if set:
    Prevents ringing the bells.
    Prevents sending telegraph packets.
    Prevents sending heartbeat packets.

  't' = Telegraph packet
  'a' = "acknowledged", response to received 't' packet
  'b' = Aborted sending packet because bouncing detected
  'i' = Aborted sending packet because 'inputLock' is true (usualy when reciving a signal)
  'e' = General error
  'h' = heartbeat, gets sent every ~1 sec
  'l' = sets 'actLock'
  'r' = release 'actLock'

  Q1 - 'telegraphSigOut'
  Q2 - 'actLock'
  Q3 - '!hasReceivedHeartbeat'

TODO:
  - OTA
  - Prevent sending packets too frequently, insted: delay packet and give a warning to operator (TXP). As opposed to current behavior of seting 'actLock'.
  - Logging
  - Remote error reporting (sms/email)
*/

bool blink = true;

uint8_t dipPins[] = { A2, A3, A4, A5, A6, A7 };  // Pins used to configure IP address (like a dip switch)

// Default IP adresses for the pair (4th octet gets overwritten in 'setup'):
IPAddress ipLocal(192, 168, 50, 2);
IPAddress ipRemote(192, 168, 50, 3);


bool isWaitingForHeartbeat = true;  // Prevents the system from doing most tasks before getting a first heartbeat
bool actLock = false;

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

  // Telegraph signal out:
  pinMode(LED_D0, OUTPUT);
  pinMode(D0, OUTPUT);

  // Act lock:
  pinMode(LED_D1, OUTPUT);
  pinMode(D1, OUTPUT);

  // Blink led is handled at start of setup.


  digitalWrite(LED_D1, actLock);  // Updates act lock leds
  digitalWrite(D1, actLock);

  digitalWrite(LED_D2, isWaitingForHeartbeat);
  digitalWrite(D2, isWaitingForHeartbeat);

  digitalWrite(BLINK_LED, LOW);  // Rapidly blink BLINK_LED at end of setup
  delay(50);
  digitalWrite(BLINK_LED, HIGH);
  Serial.println("t4");
}

void loop() {

  // Process receiving a telegraph signal (button press):
  bool telegraphSigIn = digitalRead(A0) | !digitalRead(BTN_USER);
  if (telegraphSigIn > previousTelegraphSigIn) {  // If 'telegraphSigIn' has rissen
    Serial.println(sendTelegraphPacket());        // Atepmts to send a telegraph packet, and prints the response.
  }
  previousTelegraphSigIn = telegraphSigIn;  // To detect a rising edge



  // What to do when receive a udp packet:
  uint8_t packetSize = udp.parsePacket();
  if (packetSize) {
    udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.print("Received packet: ");
    Serial.println(packetBuffer);

    if (packetSize > 1) throwError("WARN: Received a packet with a size greater then 1!");  // No packets are supposed to be more than 1 long, so if it is, something is awry.

    if (packetBuffer[0] == 'h') onReceiveHeartbeat();                                                     // Process heartbeat
    if (packetBuffer[0] == 'e' && !actLock) throwError("WARN: Paired station has experienced a error!");  // Process error pakcet exept if 'actLock' is on, to prevent the packets beeing sent back and fourth.
    if (packetBuffer[0] == 'l') throwError("WARN: Manually triggred actLock");                            // For manually triggering act lock
    if (packetBuffer[0] == 'r') releaseActLock();                                                         // For manually releasing act lock
    if (packetBuffer[0] == 't') Serial.println(onReceiveTelegraphPacket());                               // Process telegraph packet
    if (packetBuffer[0] == 'a') lastAcknowledgement = millis();                                           // Process acknowledgement packet

  }  // end 'if (pakcetSize)'

  if (millis() - lastHeartBeatSent > HEARTBEAT_RATE) {  // Sends heartbeats
    lastHeartBeatSent = millis();
    if (!actLock) udpSend('h');      // Do not send if act lock is on
    digitalWrite(BLINK_LED, blink);  // Toggle blink led
    blink = !blink;
  }


  if (!actLock && !isWaitingForHeartbeat) {                   // Only act if actLock is true, or if has not yet received its first heartbeat.
    if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME) {  // Resets 'telegraphSigOut' relay
      digitalWrite(LED_D0, LOW);
      digitalWrite(D0, LOW);
    }
    if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME + INPUTLOCK_DELAY) inputLock = false;                                           // Resets 'inputLock' after 'telegraphSigIn' relay has had time to open.
    if (millis() - lastHeartBeatReceived > MAX_TIME_NO_HEARTBEAT) throwError("WARN: Heartbeat lost!");                                    // Errors if does not receive any heatbeats in enough time
    if (millis() - lastTelegraphPacket > MAX_PING && lastTelegraphPacket > lastAcknowledgement) throwError("WARN: No acknowledgement!");  // Errors if does not receive an acknowledgement in time
  }
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
  if (actLock) return ('e');
  if (isWaitingForHeartbeat) return ('e');
  if (millis() - lastTelegraphPacket < BOUNCE_TIME) return ('b');
  if (inputLock) return ('i');
  udpSend('t');
  lastTelegraphPacket = millis();
  return ('t');
}

char onReceiveTelegraphPacket() {
  if (actLock) return ('e');
  if (isWaitingForHeartbeat) return ('e');
  if (inputLock) {
    throwError("WARN: Received two packets in too short of a time period!");
    return ('e');
  }
  inputLock = true;  // To make sure we dont process the output signal as a input signal.
  digitalWrite(LED_D0, HIGH);
  digitalWrite(D0, HIGH);
  udpSend('a');
  lastTelegraphSigOut = millis();
  return ('a');
}

void onReceiveHeartbeat() {
  lastHeartBeatReceived = millis();
  if (isWaitingForHeartbeat) {
    isWaitingForHeartbeat = false;
    digitalWrite(LED_D2, isWaitingForHeartbeat);
    digitalWrite(D2, isWaitingForHeartbeat);
  }
}

void throwError(char errorMessage[]) {
  actLock = true;
  Serial.println(errorMessage);
  udpSend('e');  // Tell the paired station that we have experienced a error
  Serial.println("Act lock has been enabled @");
  Serial.println(millis());
  digitalWrite(LED_D1, actLock);
  digitalWrite(D1, actLock);
}

void releaseActLock() {
  actLock = false;
  isWaitingForHeartbeat = true;
  Serial.println("Act lock has been remotely disabled @");
  Serial.println(millis());
  digitalWrite(LED_D1, actLock);
  digitalWrite(D1, actLock);
  digitalWrite(LED_D2, isWaitingForHeartbeat);
  digitalWrite(D2, isWaitingForHeartbeat);
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
