#define LENOF(array) (sizeof(array) / sizeof(array[0]))

#define PHONE_SIG_IN_PIN A2   // Connected to the output of the spindle
#define PHONE_SIG_OUT_PIN D2  // Connected to the phone bell
#define PHONE_SIG_OUT_PIN_LED LED_D2

#define MAX_PING 100                                  // ms, Maximum permissable delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000                           // ms, Time delay between sending heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 5000                    // ms, Time allowed without receiving heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100                          // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 20                            // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time off relays
#define BLINK_LED LED_RESET                           // pinId, The led used for indicating a local heartbeat
#define BOUNCE_TIME 20                                // ms, maximum possible bounce time for inputs
#define PHONE_SIG_THRESHOLD 100                       // (5 / 1024 V), Threshold for determening whether phone signal is high or low.
#define PHONE_SIG_SAMPLE_RATE 50                      // ms, Sample rate for phone signal.
#define TELEGRAPH_SIG_IN_DURATION_FOR_ERROR_ACK 5000  // ms, duration of telegraph sig in for it to count as eorror ack.
#define WARNING_BELL_RATE 500                         // ms, rate of rings for warning of errors.

#define PORT 8888  // Port that the controllers send and listen on.

#include "library.c"

#include <Ethernet.h>
#include <EthernetUdp.h>


/* Documentaion

  Signal - The electrical signal that we are simulating.
  Packet - The udp packet that travels between stations.
  Controller - The Arduino Opta plc that runs this code. Each station has 1 or 2 controllers.
  (controller) ID - uniqe ID/iterator for each controller. This ID starts at 3 for the eastern most controller, and iterates westwords through all controllers.
  Phone spindle - Hand crack that operator spins in order to dial the code for another station.

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
  - Change IP system to allow each station (not controller or pair) to be on diffrent layer 3 networks.
  - Insted of setting a warning lamp on errors, insted ring the bells 6 times repetedly.
  - Broadcast phone ringing signals to all stations.

  - Fix bug where t-sig can ack error instanly if recives t-pkt at the same time as rising edge.
*/

bool blink = true;

uint8_t dipPins[] = { A3, A4, A5, A6, A7 };  // Pins used to configure IP address (like a dip switch)

enum controllers {  //
  null,
  testA,
  testB,
  garnesA,
  garnesB,
  arnaA,
  arnaB,
  haukelandA,
  haukelandB
};

IPAddress ipLocal;
IPAddress ipRemote;

IPAddress ipAddresses[] = {
  { 0, 0, 0, 0 },        // Null IP address
  { 192, 168, 40, 60 },  // Test A (to TestB)
  { 192, 168, 40, 61 },  // Test B (to TestA)
  { 192, 168, 50, 60 },  // Garnes A (to Haukeland B)
  { 192, 168, 50, 61 },  // Garnes B (to Arna A)
  { 192, 168, 51, 60 },  // Arna A (to Garnes B)
  { 192, 168, 51, 62 },  // Arna B (to Haukeland A)
  { 192, 168, 52, 60 },  // Haukeland A (to Arna B)
  { 192, 168, 52, 61 }   // Haulekand B (to Garnes A)
};

const uint8_t pairedControllers[][2]{ // A table to define pairs
                                      { null, null },
                                      { testA, testB },
                                      { garnesB, arnaA },
                                      { arnaB, haukelandA },
                                      { haukelandB, garnesA }
};  // (Dont argue with the linter)


bool warningLamp = true;

bool previousTelegraphSigIn;
bool inputLock;
bool previousPhoneSigIn;
bool phoneSigOut;

unsigned long lastTelegraphPacket;
unsigned long lastAcknowledgement;
uint16_t unAcknowledgedTelegraphPackets;
uint8_t remainingErrorRings;  // How many rings of the bell remains for warning of an error
unsigned long lastPhoneSigSample;

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
  Serial.print("dipValue = ");
  Serial.println(dipValue);

  if (dipValue > LENOF(ipAddresses)) {
    dipValue = 0;
    throwError("WARN: 'dipValue' out of bounds!");
  }

  ipLocal = ipAddresses[dipValue];               // Get local IP
  ipRemote = ipAddresses[findPaired(dipValue)];  // Get remote IP

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

  // Phone in/out:
  pinMode(PHONE_SIG_IN_PIN, INPUT);
  pinMode(PHONE_SIG_OUT_PIN, OUTPUT);
  pinMode(PHONE_SIG_OUT_PIN_LED, OUTPUT);

  udpSendAll(analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD ? 'P' : 'p');  // Send 'P' if phone spindle is spinning and 'p' if not spinning.
  if (analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD) throwError("WARN: Phone signal in is high at boot!");

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

  // Process receiving a phone signal (spindle):
  bool phoneSigIn;
  if (millis() - lastPhoneSigSample > PHONE_SIG_SAMPLE_RATE) {  // Sample phone signal evry ~50ms
    phoneSigIn = analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD;
    if (phoneSigIn != previousPhoneSigIn) {  // If 'phoneSigIn' has changed
      udpSendAll(phoneSigIn ? 'P' : 'p');    // Send 'P' if phone spindle has started spinning and 'p' if has stopped spinning.
      lastPhoneSigSample = millis();
    }
    previousPhoneSigIn = phoneSigIn;  // To detect a rising edge
  }

  // Process 'errorAckBtn' press:
  if (digitalRead(A1) || millis() - lastTelegraphPacket > TELEGRAPH_SIG_IN_DURATION_FOR_ERROR_ACK && telegraphSigIn && !inputLock) onErrorAck();  // Triggers 'onErrorAck' if error ack button is pressed



  // What to do when a udp packet is received:
  uint8_t packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet ");
    Serial.print(packetSize);
    Serial.print(udp.read(packetBuffer, packetSize));
    Serial.print(": ");
    Serial.println(packetBuffer);

    if (packetSize > 1) throwError("WARN: Received a packet with a size greater then 1!");  // No packets are supposed to be more than 1 long, so if it is, something is awry.

    if (packetBuffer[0] == 'h') onReceiveHeartbeat();                                         // Process heartbeat
    if (packetBuffer[0] == 'e') throwError("WARN: Paired station has experienced a error!");  // Process error packet
    if (packetBuffer[0] == 't') Serial.println(onReceiveTelegraphPacket());                   // Process telegraph packet
    if (packetBuffer[0] == 'a') onRecieveAcknowledgement();                                   // Process acknowledgement packet
    if (packetBuffer[0] == 'P') phoneSigOut = true;                                           // Close phone bell relay
    if (packetBuffer[0] == 'p') phoneSigOut = false;                                          // Open phone bell relay
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
  if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME + INPUTLOCK_DELAY) inputLock = false;                                                     // Resets 'inputLock' after 'telegraphSigIn' relay has had time to open.
  if (millis() - lastHeartBeatReceived > MAX_TIME_NO_HEARTBEAT && !warningLamp) throwError("WARN: Heartbeat lost!");                              // Errors if does not receive any heatbeats in enough time
  if (millis() - lastTelegraphPacket > MAX_PING && unAcknowledgedTelegraphPackets != 0 && !warningLamp) throwError("WARN: No acknowledgement!");  // Errors if does not receive an acknowledgement in time
  if (millis() - lastTelegraphSigOut > WARNING_BELL_RATE && remainingErrorRings > 0) {
    sendTelegraphSignal();
    remainingErrorRings--;
    Serial.print("'ramainingErrorRings' = ");
    Serial.println(remainingErrorRings);
  }

  digitalWrite(PHONE_SIG_OUT_PIN, phoneSigOut);  // Update pins to reflect internal state variables
  digitalWrite(PHONE_SIG_OUT_PIN_LED, phoneSigOut);
}


void udpSend(char code) {
  Serial.print("Sent packet ");
  Serial.print(udp.beginPacket(ipRemote, PORT));
  Serial.print(udp.write(code));
  Serial.print(udp.endPacket());
  Serial.print(": ");
  Serial.println(code);
}

void udpSendAll(char code) {
  Serial.print("Sent packet: ");
  Serial.print(code);
  Serial.print("; ");
  for (uint8_t i = 1; i < LENOF(ipAddresses); i++) {
    Serial.print(udp.beginPacket(ipAddresses[i], PORT));
    Serial.print(udp.write(code));
    Serial.print(udp.endPacket());
    Serial.print("; ");
  }
  Serial.println();
}

char sendTelegraphPacket() {
  if (millis() - lastTelegraphPacket < BOUNCE_TIME) return ('b');
  if (inputLock) return ('i');
  unAcknowledgedTelegraphPackets++;
  Serial.println(unAcknowledgedTelegraphPackets);
  udpSend('t');
  lastTelegraphPacket = millis();
  return ('t');
}

char onReceiveTelegraphPacket() {
  if (inputLock) {
    throwError("WARN: Received two packets in too short of a time period!");
    return ('e');
  }
  sendTelegraphSignal();
  udpSend('a');
  return ('a');
}

void sendTelegraphSignal() {
  inputLock = true;  // To make sure we do not process the output signal as a input signal.
  digitalWrite(LED_D0, HIGH);
  digitalWrite(D0, HIGH);
  lastTelegraphSigOut = millis();
}

void onRecieveAcknowledgement() {
  lastAcknowledgement = millis();
  unAcknowledgedTelegraphPackets--;
  Serial.println(unAcknowledgedTelegraphPackets);
}

void onReceiveHeartbeat() {
  lastHeartBeatReceived = millis();
}

void throwError(char errorMessage[]) {
  if (!warningLamp) {
    udpSend('e');  // Tell the paired station that we have experienced an error
    Serial.print("'ramainingErrorRings' = ");
    remainingErrorRings = 9;
    Serial.println(remainingErrorRings);
  }
  warningLamp = true;
  Serial.println(errorMessage);
  Serial.println("There has been a error @");
  Serial.println(millis());
  digitalWrite(LED_D1, warningLamp);
  digitalWrite(D1, warningLamp);
}

void onErrorAck() {
  if (!warningLamp) return;
  warningLamp = false;
  unAcknowledgedTelegraphPackets = 0;
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

uint8_t findPaired(uint8_t localID) {  // Figure out the ID of the controller in the same pair as this controller
  Serial.print("LocalID: ");
  Serial.println(localID);  // 'dipValue'

  for (uint8_t i1 = 0; i1 < LENOF(pairedControllers); i1++) {  // for each pair
    Serial.print("t5:");
    Serial.println(i1);

    for (bool i2 = 0;; i2++) {  // for each controller in a pair
      Serial.print("t6:");
      Serial.println(i2);

      Serial.print("t7:");
      Serial.println(pairedControllers[i1][i2]);

      if (pairedControllers[i1][i2] == localID) {
        Serial.print("t8:");
        Serial.print(i1);
        Serial.print(":");
        Serial.print(i2);
        Serial.print(":");
        Serial.println(!i2);
        return (pairedControllers[i1][!i2]);
      }
      if (i2) break;  // if has done 2. loop
    }
  }
  return (0);
}
