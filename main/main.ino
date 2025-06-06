#define LENOF(array) (sizeof(array) / sizeof(array[0]))


#define TELEGRAPH_SIG_IN_PIN A0       // pinID, digital input used to listen on the telegraph wire.
#define TELEGRAPH_SIG_OUT_RLY D0      // pinID, relay output used to send a signal on the telegraph wire.
#define TELEGRAPH_SIG_OUT_LED LED_D0  // pinID, relay output used to send a signal on the telegraph wire.
#define ERROR_ACK_PIN A1              // pinID, digital input to ack a error.
#define ERROR_LOCAL_RLY D1            // pinID, relay output used to signal that controller is in error state.
#define ERROR_LOCAL_LED LED_D1        // pinID, relay output used to signal that controller is in error state.
#define PHONE_SIG_IN_PIN A2           // pinID, Connected to the output of the spindle.
#define PHONE_SIG_OUT_RLY D2          // pinID, Connected to the phone bell
#define PHONE_SIG_OUT_LED LED_D2      // pinID
#define BLINK_LED LED_RESET           // pinId, The led used for indicating a local heartbeat


#define MAX_PING 100                // ms, Maximum permissable delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000         // ms, Time delay between sending heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 5000  // ms, Time allowed without receiving heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100        // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 20          // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time off relays
#define BOUNCE_TIME 20              // ms, maximum possible bounce time for inputs
#define PHONE_SIG_THRESHOLD 100     // (5 / 1024 V), Threshold for determening whether phone signal is high or low.
#define PHONE_SIG_SAMPLE_RATE 50    // ms, Sample rate for phone signal.
#define PACKET_MAX_SIZE 1           // bytes, how many bytes we allow to parce for recieving UDP packets.

#define PORT 8888  // Port that the controllers send and listen on.

#include "library.c"

#include <Ethernet.h>
#include <EthernetUdp.h>


/* Documentaion

  Terms:
    Signal - The electrical signal that we are simulating.
    Packet - The udp packet that travels between stations.
    Telegraph button - The button that TXP/operator presses to send signals to the paired station.
    Controller - The Arduino Opta plc that runs this code. Each station has 1 or 2 controllers.
    (controller) ID - uniqe ID/iterator for each controller. This ID starts at 3 for the eastern most controller, and iterates westwords through all controllers.
    Phone spindle - Hand crack that operator spins in order to dial the code for another station.
    timestamp variable - Variables of type 'unsined long' that stores the return values of 'millis()'
    Remote server - A remote server that recievs heartbeats and error messages from all controllers.

  Codes:
    't' = Telegraph packet
    'a' = "acknowledged", response to received 't' packet
    'b' = Aborted sending packet because bouncing detected
    'i' = Aborted sending packet because 'inputLock' is true (usually when receiving a signal)
    'e' = General error
    'h' = heartbeat, gets sent every ~1 sec
    'P'/'p' = Phone packets, capital case for rising, and lower case for falling.

  I/O:
    I1 - 'telegraphSigIn'
    I2 - 'onErrorAck()'
    I3 - 'phoneSigIn'
    I4...I8 - Confuguration dip pins
    Q1 - 'telegraphSigOut'
    Q2 - 'errorLocal'
    Q3 - 'phoneSigOut'
    Q4 - unused

TODO:
  - OTA
  - Prevent sending packets too frequently, insted: delay packet and give a warning to operator (TXP). As opposed to current behavior of seting 'actLock'.
  - Logging
  - Remote error reporting (sms/email)
  - Handle 'millis' overflow
  - Add outputs for errorRemote and errorLocal, and ackAll and ackLocal inputs
  - Make send 'H' if not in error and 'h' if in error
  - Make phone signal send 'P' continiustly whilst active, insted of toggling with 'P' and 'p'
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
  haukelandB,
  testC,
  testD
};

uint8_t localID;
char localIDChar[33];  // Seemed to work with a size of 0, but im scared it might be overflowing and using memory addresses it shouldent, witch might cause other problems later.
uint8_t pairedID;

IPAddress ipAddresses[] = {
  { 0, 0, 0, 0 },       // Null IP address
  { 10, 3, 2, 50 },     // Test A (to TestB)
  { 10, 3, 2, 51 },     // Test B (to TestA)
  { 10, 1, 2, 60 },     // Garnes A (to Haukeland B)
  { 10, 1, 2, 61 },     // Garnes B (to Arna A)
  { 10, 2, 2, 60 },     // Arna A (to Garnes B)
  { 10, 2, 2, 62 },     // Arna B (to Haukeland A)
  { 10, 3, 2, 60 },     // Haukeland A (to Arna B)
  { 10, 3, 2, 61 },     // Haulekand B (to Garnes A)
  { 192, 168, 1, 50 },  // Test C (to testD)
  { 192, 168, 1, 51 }   // Test D (to testC)
};

IPAddress gateways[] = {
  { 0, 0, 0, 0 },      // Null
  { 10, 3, 2, 1 },     // Test A
  { 10, 3, 2, 1 },     // Test B
  { 10, 1, 2, 1 },     // Garnes A
  { 10, 1, 2, 1 },     // Garnes B
  { 10, 2, 2, 1 },     // Arna A
  { 10, 2, 2, 1 },     // Arna B
  { 10, 3, 2, 1 },     // Haukeland A
  { 10, 3, 2, 1 },     // Haulekand B
  { 192, 168, 1, 1 },  // Test C
  { 192, 168, 1, 1 }   // Test D
};

byte macAddresses[][6] = {
  // 'Ethernet.begin()' wants a MAC address as a argument, but it dosent seem to acually want to use it, as 'Ethernet.MACAddress()' returns something else.
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x00 },  // Null
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x01 },  // Test A
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x02 },  // Test B
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x03 },  // Garnes A
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x04 },  // Garnes B
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x05 },  // Arna A
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x06 },  // Arna B
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x07 },  // Haukeland A
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x08 },  // Haukeland B
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x09 },  // Test C
  { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x0a }   // Test E
};

IPAddress remoteServerIp = { 172, 30, 1, 12 };  // IP address, IP address of a remote server to also recive heartbeats.

const uint8_t pairedControllers[][2]{ // A table to define pairs
                                      { null, null },
                                      { testA, testB },
                                      { garnesB, haukelandA},
                                      { arnaB, arnaA },
                                      { haukelandB, garnesA },
                                      { testC, testD }
};  // (Dont argue with the linter)


bool errorLocal = true;

bool previousTelegraphSigIn;
bool inputLock;
bool previousPhoneSigIn;
bool phoneSigOut;

unsigned long lastTelegraphPacket;
unsigned long lastAcknowledgement;
uint16_t unAcknowledgedTelegraphPackets;
unsigned long lastPhoneSigSample;

unsigned long lastTelegraphSigOut;

unsigned long lastHeartBeatReceived;
unsigned long lastHeartBeatSent;




EthernetUDP udp;
char packetBuffer[PACKET_MAX_SIZE + 1];  // Buffer to hold incoming packet, "+ 1" is to make space for null character.


void setup() {
  pinMode(BLINK_LED, OUTPUT);
  digitalWrite(BLINK_LED, HIGH);  // Enable BLINK_LED led at start of setup

  Serial.begin(9600);
  delay(2000);  // To give time for serial to establish.
  Serial.println("\nSerial begun.");


  // Figure out ID of paired controller:
  initDipPins(dipPins, LENOF(dipPins));
  Serial.print("'dipPins': ");
  uint8_t dipValue = readDipPins(dipPins, LENOF(dipPins));  // Decimal interpretation of dip pins
  Serial.print("'dipValue' = ");
  Serial.println(dipValue);
  localID = dipValue;  // Sets the local id before checks, so it can be overwritten if they fail.

  if (LENOF(ipAddresses) + LENOF(gateways) + LENOF(macAddresses) == LENOF(ipAddresses) * 3) {
    Serial.print("PASS: All lookup tables have the same number of eneries: ");
    Serial.println(LENOF(ipAddresses));
  } else {
    localID = 0;
    throwError("FAIL: not all lookup tables have the same value.");
    Serial.print("'ipAddresses' = ");
    Serial.println(LENOF(ipAddresses));
    Serial.print("'gateways' = ");
    Serial.println(LENOF(gateways));
    Serial.print("'macAddresses' = ");
    Serial.println(LENOF(macAddresses));
  }

  if (dipValue <= LENOF(ipAddresses) - 1) {
    Serial.print("PASS: 'dipValue' is within bounds: ");
    Serial.print(dipValue);
    Serial.print(" / ");
    Serial.println(LENOF(ipAddresses) - 1);
  } else {
    localID = 0;
    throwError("FAIL: 'localID' out of bounds! (or 'ipAddresses' has too few entries)");
  }

  // 'dipValue' should not be used below here.


  Serial.print("'localID' = ");
  Serial.println(localID);

  sprintf(localIDChar, "%d", localID);
  Serial.print("'localIDChar = \"");
  Serial.print(localIDChar);
  Serial.println("\"");

  pairedID = findPaired(localID);
  Serial.print("'pairedID' = ");
  Serial.println(pairedID);


  // Ethernet/Udp:
  Ethernet.begin(macAddresses[localID], ipAddresses[localID], gateways[localID], gateways[localID]);
  Serial.println("Ethernet begun.");

  if (Ethernet.hardwareStatus() != EthernetNoHardware) {
    Serial.print("PASS: Ethernet hardware found: ");
  } else {
    throwError("FAIL: Ethernet hardware not found.");
  }
  Serial.println(Ethernet.hardwareStatus());

  if (Ethernet.linkStatus() != LinkOFF) {
    Serial.print("PASS: Ethernet link is not off: ");
  } else {
    throwError("FAIL: Ethernet link is off: ");
  }
  Serial.println(Ethernet.linkStatus());

  Serial.print("\nLocal IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Paried IP: ");
  Serial.println(ipAddresses[pairedID]);
  Serial.print("Remote server IP: ");
  Serial.println(remoteServerIp);
  Serial.print("Port: ");
  Serial.println(PORT);
  Serial.print("MAC: ");
  Serial.println(Ethernet.macAddress());
  Serial.print("DNS: ");
  Serial.println(Ethernet.dnsServerIP());
  Serial.print("Gateway: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.println();

  udp.begin(PORT);
  Serial.println("Udp begun.");

  Serial.print("Pin modes: ");

  // Telegraph signal in:
  verbosePinMode(TELEGRAPH_SIG_IN_PIN, INPUT);
  verbosePinMode(BTN_USER, INPUT);

  // Telegraph signal out:
  verbosePinMode(TELEGRAPH_SIG_OUT_RLY, OUTPUT);
  verbosePinMode(TELEGRAPH_SIG_OUT_LED, OUTPUT);

  // Error acknowledge button:
  verbosePinMode(ERROR_ACK_PIN, INPUT);

  // Error local signal:
  verbosePinMode(ERROR_LOCAL_RLY, OUTPUT);
  verbosePinMode(ERROR_LOCAL_LED, OUTPUT);

  // Phone in/out:
  verbosePinMode(PHONE_SIG_IN_PIN, INPUT);
  verbosePinMode(PHONE_SIG_OUT_RLY, OUTPUT);
  verbosePinMode(PHONE_SIG_OUT_LED, OUTPUT);

  Serial.println();

  udpSendAll(analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD ? 'P' : 'p');  // Send 'P' if phone spindle is spinning and 'p' if not spinning.
  if (analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD) throwError("WARN: Phone signal in is high at boot!");

  // Blink led is handled at start of setup.

  digitalWrite(ERROR_LOCAL_RLY, errorLocal);  // Updates error local signal
  digitalWrite(ERROR_LOCAL_LED, errorLocal);

  digitalWrite(BLINK_LED, LOW);  // Rapidly blink BLINK_LED at end of setup
  delay(50);
  digitalWrite(BLINK_LED, HIGH);
  Serial.println("Setup finished!\n");
}

void loop() {

  // Process receiving a telegraph signal (button press):
  bool telegraphSigIn = digitalRead(TELEGRAPH_SIG_IN_PIN) | !digitalRead(BTN_USER);
  if (telegraphSigIn > previousTelegraphSigIn) {  // If 'telegraphSigIn' has risen
    sendTelegraphPacket();                        // Attemps to send a telegraph packet.
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
  if (digitalRead(ERROR_ACK_PIN)) onErrorAck();  // Triggers 'onErrorAck' if error ack button is pressed



  // What to do when a udp packet is received:
  uint16_t packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print(" Rx: ");
    udp.read(packetBuffer, PACKET_MAX_SIZE);
    packetBuffer[packetSize > PACKET_MAX_SIZE ? PACKET_MAX_SIZE : packetSize] = '\0';  // Termintaes the c-string with a null, so the message isnt polluted by previous messages.
    Serial.println(packetBuffer);

    if (packetSize > 1) {
      throwError("WARN: Received a packet with a size greater then 1!");  // No packets are supposed to be more than 1 long, so if it is, something is awry.
      Serial.print("  Previous packet size: ");
      Serial.println(packetSize);
      if (packetSize == 508) Serial.println("  Note: 508 is the maximum size of pakcets supported by EthernetUDP.");
    }

    switch (packetBuffer[0]) {
      case 'h':
        onReceiveHeartbeat();  // Process heartbeat
        break;
      case 'e':
        throwError("WARN: Paired station has experienced a error!");  // Process error packet
        break;
      case 't':
        onReceiveTelegraphPacket();  // Process telegraph packet
        break;
      case 'a':
        onRecieveAcknowledgement();  // Process acknowledgement packet
        break;
      case 'P':
        phoneSigOut = true;  // Close phone bell relay
        break;
      case 'p':
        phoneSigOut = false;  // Open phone bell relay
        break;
      default:
        throwError("WARN: Recived a invalid packet!");
        Serial.print("  'packetBuffer[");
        Serial.print(sizeof(packetBuffer));
        Serial.print("]' = ");
        Serial.println(packetBuffer);
    }
  }  // end 'if (packetSize)'

  if (millis() - lastHeartBeatSent > HEARTBEAT_RATE) {  // Sends heartbeats
    lastHeartBeatSent = millis();
    udpSend('h');
    digitalWrite(BLINK_LED, blink);  // Toggle blink led
    blink = !blink;
    udp.beginPacket(remoteServerIp, PORT);
    udp.write("h ");
    udp.write(localIDChar);
    udp.write('\n');
    udp.endPacket();
  }
  if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME) {  // Resets 'telegraphSigOut' relay
    digitalWrite(TELEGRAPH_SIG_OUT_RLY, LOW);
    digitalWrite(TELEGRAPH_SIG_OUT_LED, LOW);
  }
  if (millis() - lastTelegraphSigOut > T_SIGNAL_ON_TIME + INPUTLOCK_DELAY) inputLock = false;                                                    // Resets 'inputLock' after 'telegraphSigIn' relay has had time to open.
  if (millis() - lastHeartBeatReceived > MAX_TIME_NO_HEARTBEAT && !errorLocal) throwError("WARN: Heartbeat lost!");                              // Errors if does not receive any heatbeats in enough time
  if (millis() - lastTelegraphPacket > MAX_PING && unAcknowledgedTelegraphPackets != 0 && !errorLocal) throwError("WARN: No acknowledgement!");  // Errors if does not receive an acknowledgement in time

  digitalWrite(PHONE_SIG_OUT_RLY, phoneSigOut);  // Update pins to reflect internal state variables
  digitalWrite(PHONE_SIG_OUT_LED, phoneSigOut);
}


void udpSend(char code) {
  Serial.print("Tx: ");
  udp.beginPacket(ipAddresses[pairedID], PORT);
  udp.write(code);
  udp.endPacket();
  Serial.println(code);
}

void udpSendAll(char code) {
  Serial.print("Tx(all): ");
  for (uint8_t i = 1; i < LENOF(ipAddresses); i++) {
    udp.beginPacket(ipAddresses[i], PORT);
    udp.write(code);
    udp.endPacket();
  }
  Serial.println(code);
}

void sendTelegraphPacket() {
  if (millis() - lastTelegraphPacket < BOUNCE_TIME) return;
  if (inputLock) return;
  lastTelegraphPacket = millis();
  unAcknowledgedTelegraphPackets++;
  Serial.println("Sending telegraph packet!");
  Serial.print("  'unAcknowledgedTelegraphPackets' = ");
  Serial.println(unAcknowledgedTelegraphPackets);
  udpSend('t');
}

void onReceiveTelegraphPacket() {
  if (inputLock) {
    throwError("WARN: Received telegraph packet whilst 'inputLock' is active!");
    return;
  }
  sendTelegraphSignal();
  Serial.println("Recived telegraph packet!");
  Serial.println("  Sending acknowledgement!");
  udpSend('a');
}

void sendTelegraphSignal() {
  inputLock = true;  // To make sure we do not process the output signal as a input signal.
  digitalWrite(TELEGRAPH_SIG_OUT_RLY, HIGH);
  digitalWrite(TELEGRAPH_SIG_OUT_LED, HIGH);
  lastTelegraphSigOut = millis();
}

void onRecieveAcknowledgement() {
  lastAcknowledgement = millis();
  unAcknowledgedTelegraphPackets--;
  Serial.println("Recived acknowledgement!");
  Serial.print("  'unAcknowledgedTelegraphPackets' = ");
  Serial.println(unAcknowledgedTelegraphPackets);
}

void onReceiveHeartbeat() {
  lastHeartBeatReceived = millis();
}

void throwError(char errorMessage[]) {
  if (!errorLocal) udpSend('e');  // Tell the paired station that we have experienced an error
  errorLocal = true;
  Serial.print(errorMessage);
  Serial.print("\n  There has been a error @: ");
  Serial.println(millis());
  digitalWrite(ERROR_LOCAL_RLY, errorLocal);
  digitalWrite(ERROR_LOCAL_LED, errorLocal);
}

void onErrorAck() {
  if (!errorLocal) return;
  errorLocal = false;
  unAcknowledgedTelegraphPackets = 0;
  Serial.print("Error has been acknowledged @");
  Serial.println(millis());
  digitalWrite(ERROR_LOCAL_RLY, errorLocal);
  digitalWrite(ERROR_LOCAL_LED, errorLocal);
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
    Serial.print(digitalRead(pins[i]));
    Serial.print(", ");
  }
  return value;
}

uint8_t findPaired(uint8_t localID) {                          // Figure out the ID of the controller in the same pair as this controller
  for (uint8_t i1 = 0; i1 < LENOF(pairedControllers); i1++) {  // for each pair

    for (bool i2 = 0;; i2++) {  // for each controller in a pair
      if (pairedControllers[i1][i2] == localID) {
        return (pairedControllers[i1][!i2]);
      }
      if (i2) break;  // if has done 2. loop
    }
  }
  return (0);
}

void verbosePinMode(pin_size_t pin, PinMode mode) {
  Serial.print(pin);
  Serial.print(" > ");
  Serial.print(mode);
  Serial.print("; ");
  pinMode(pin, mode);
}
