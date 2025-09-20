#define LENOF(array) (sizeof(array) / sizeof(array[0]))


#define TELEGRAPH_SIG_IN_PIN A0       // pinID, digital input used to listen on the telegraph wire.
#define TELEGRAPH_SIG_OUT_RLY D0      // pinID, relay output used to send a signal on the telegraph wire.
#define TELEGRAPH_SIG_OUT_LED LED_D0  // pinID, relay output used to send a signal on the telegraph wire.
#define ERROR_LOCAL_ACK_PIN A1        // pinID, digital input to ack a error.
#define ERROR_LOCAL_RLY D1            // pinID, relay output used to signal that controller is in error state.
#define ERROR_LOCAL_LED LED_D1        // pinID, relay output used to signal that controller is in error state.
#define PHONE_SIG_IN_PIN A2           // pinID, Connected to the output of the spindle.
#define PHONE_SIG_OUT_RLY D2          // pinID, Connected to the phone bell
#define PHONE_SIG_OUT_LED LED_D2      // pinID
#define ERROR_REMOTE_ACK_PIN A3       // pinID, pin to ack errors on all controllers on the line.
#define ERROR_REMOTE_RLY D3           // pinID, Temporary output for remote error, final sollution will be on Q2, and differentiate with fast/slow blinks
#define ERROR_REMOTE_LED LED_D3       // pinID
#define BLINK_LED LED_RESET           // pinId, The led used for indicating a local heartbeat


#define MAX_PING 500                // ms, Maximum permissable delay from sending a telegraph packet to receive an acknowledgement.
#define HEARTBEAT_RATE 1000         // ms, Time delay between sending heartbeat packets.
#define MAX_TIME_NO_HEARTBEAT 5000  // ms, Time allowed without receiving heartbeat packets before locking.
#define T_SIGNAL_ON_TIME 100        // ms, Time delay between closing and opening 'telegraphSigOut' relay
#define INPUTLOCK_DELAY 20          // ms, The time where 'inputLock' is true but 'telegraphSigOut' is false, to account for off-time off relays
#define BOUNCE_TIME 20              // ms, maximum possible bounce time for inputs
#define PHONE_SIG_THRESHOLD 100     // (5 / 1024 V), Threshold for determening whether phone signal is high or low.
#define PHONE_SIG_SAMPLE_RATE 50    // ms, Sample rate for phone signal.
#define PACKET_MAX_SIZE 2           // bytes, how many bytes we allow to parce for recieving UDP packets.

#define PORT 8888  // Port that the controllers send and listen on.

#include "library.c"

#include <Ethernet.h>
#include <EthernetUdp.h>

#include CONFIG_PATH

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
    Line - Term for the intire telegraph line, from end station to end station. All controllers on the same "line" share error remote.
      There is to be made no connections betweenm lines, only TXPs/operators can transfer information between lines.
    Phone line - Term for the phone line, can include fewer controllers then line, but should not include any controllers from outside the same line.
      IE, a line can have multiple phone lines. If a phone line goes beond the reaches of the line, there is no garantee of signal integrity.
    Error local - A boolean state variable that indicates whether or not this controller has expirienced a error.
    Error remote / Error line - A boolean state variable that indicates whether or not any controllers on the line has expirienced a error.

  Codes:
    't' = Telegraph packet
    'a' = "acknowledged", response to received 't' packet
    'b' = Aborted sending packet because bouncing detected
    'i' = Aborted sending packet because 'inputLock' is true (usually when receiving a signal)
    'e' = General error
    'r' = Release error / Error acknowledged on intire line
    'H'/'h' = Healthy/unhealthy heartbeat, gets sent every ~1 sec, if controller is not or is in in error local.
    'P'/'p' = Phone packets, capital case for rising, and lower case for falling.

  I/O:
    I1 - 'telegraphSigIn'
    I2 - 'ackErrorLocal'
    I3 - 'phoneSigIn'
    I4 - 'ackErrorLine'
    Q1 - 'telegraphSigOut'
    Q2 - 'errorLocal'
    Q3 - 'phoneSigOut'
    Q4 - 'errorRemote'

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

char localIDChar[33];  // Seemed to work with a size of 0, but im scared it might be overflowing and using memory addresses it shouldent, witch might cause other problems later.


bool errorLocal = true;
bool errorRemote = true;

bool previousTelegraphSigIn;
bool inputLock;
bool previousPhoneSigIn;
bool phoneSigOut;

unsigned long lastTelegraphPacket;
unsigned long lastAcknowledgement;
uint16_t unAcknowledgedTelegraphPackets;
unsigned long lastPhoneSigSample;

unsigned long lastTelegraphSigOut;

unsigned long lastHeartBeatsReceived[lineLength];
bool heartBeatStati[lineLength];  // True is healthy per controller.
unsigned long lastHeartBeatSent;




EthernetUDP udp;
char packetBuffer[PACKET_MAX_SIZE + 1];  // Buffer to hold incoming packet, "+ 1" is to make space for null character.


void setup() {
  pinMode(BLINK_LED, OUTPUT);
  digitalWrite(BLINK_LED, HIGH);  // Enable BLINK_LED led at start of setup

  Serial.begin(9600);
  delay(2000);  // To give time for serial to establish.
  Serial.println("\nSerial begun.");

  Serial.print("'localID' = ");
  Serial.println(localID);

  sprintf(localIDChar, "%d", localID);
  Serial.print("'localIDChar = \"");
  Serial.print(localIDChar);
  Serial.println("\"");

  Serial.print("'pairedID' = ");
  Serial.println(pairedID);

  Serial.print("'lineName' = ");
  Serial.println(lineName);

  Serial.print("'lineLenght' = ");
  Serial.println(lineLength);


  // Ethernet/Udp:
  Ethernet.begin(macAddress, ipAddresses[localID], localGateway, localGateway);
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

  // Error local:
  verbosePinMode(ERROR_LOCAL_ACK_PIN, INPUT);
  verbosePinMode(ERROR_LOCAL_RLY, OUTPUT);
  verbosePinMode(ERROR_LOCAL_LED, OUTPUT);

  // Phone in/out:
  verbosePinMode(PHONE_SIG_IN_PIN, INPUT);
  verbosePinMode(PHONE_SIG_OUT_RLY, OUTPUT);
  verbosePinMode(PHONE_SIG_OUT_LED, OUTPUT);

  // Error remote:
  verbosePinMode(ERROR_REMOTE_ACK_PIN, INPUT);
  verbosePinMode(ERROR_LOCAL_RLY, OUTPUT);
  verbosePinMode(ERROR_LOCAL_LED, OUTPUT);

  Serial.println();

  udpSendLine(analogRead(PHONE_SIG_IN_PIN) > PHONE_SIG_THRESHOLD ? 'P' : 'p');  // Send 'P' if phone spindle is spinning and 'p' if not spinning.
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
      udpSendLine(phoneSigIn ? 'P' : 'p');   // Send 'P' if phone spindle has started spinning and 'p' if has stopped spinning.
      lastPhoneSigSample = millis();
    }
    previousPhoneSigIn = phoneSigIn;  // To detect a rising edge
  }

  // Process 'errorAckBtn' press:
  if (digitalRead(ERROR_LOCAL_ACK_PIN)) errorLocalAck();  // Triggers 'onErrorAck' if error ack button is pressed
  if (digitalRead(ERROR_REMOTE_ACK_PIN)) errorRemoteAck();



  // What to do when a udp packet is received:
  uint16_t packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print(" Rx: ");
    Serial.print(packetSize);
    Serial.print(": ");
    udp.read(packetBuffer, PACKET_MAX_SIZE);
    packetBuffer[packetSize > PACKET_MAX_SIZE ? PACKET_MAX_SIZE : packetSize] = '\0';  // Termintaes the c-string with a null, so the message isnt polluted by previous messages.

    for (uint16_t byte = 0; byte < packetSize; byte++) Serial.print(packetBuffer[byte]);
    Serial.print("\n");

    if (packetSize > PACKET_MAX_SIZE) {
      throwError("WARN: Received a overzie packet!");
      Serial.print("  Previous packet size: ");
      Serial.println(packetSize);
      if (packetSize == 508) Serial.println("  Note: 508 is the maximum size of pakcets supported by EthernetUDP.");
    }

    switch (packetBuffer[0]) {
      case 'H':
        onReceiveHeartbeat(true, packetSize);  // Process healthy heartbeat
        break;
      case 'h':
        onReceiveHeartbeat(false, packetSize);  // Process unhealthy heartbeat
        break;
      case 'e':
        throwError("WARN: Paired station has experienced a error!");  // Process error packet
        break;
      case 'r':
        errorLocalAck();
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
    }
  }  // end 'if (packetSize)'

  if (millis() - lastHeartBeatSent > HEARTBEAT_RATE) {  // Sends heartbeats
    lastHeartBeatSent = millis();
    udpSendLine(errorLocal ? 'h' : 'H');
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
  if (millis() - lastTelegraphPacket > MAX_PING && unAcknowledgedTelegraphPackets != 0 && !errorLocal) throwError("WARN: No acknowledgement!");  // Errors if does not receive an acknowledgement in time


  errorRemote = false;                                                               // 'errorRemote' gets set to false in following for loop if any controllers report unhealthy.
  for (uint8_t lineController = 0; lineController < lineLength; lineController++) {  // Check heartbeats
    if (lineController == localID) continue;

    if (millis() - lastHeartBeatsReceived[lineController] > MAX_TIME_NO_HEARTBEAT && !errorLocal) {
      throwError("WARN: Heartbeat lost!");  // Errors if does not receive any heatbeats in enough time
      Serial.print("  last heartbeat from 'lineControllers[");
      Serial.print(lineController);
      Serial.print("]' was '");
      Serial.print(millis() - lastHeartBeatsReceived[lineController]);
      Serial.println("'ms ago.");
    };
    if (heartBeatStati[lineController] == false) errorRemote = true;
  }


  digitalWrite(ERROR_LOCAL_RLY, errorLocal);  // Update pins to reflect internal state variables
  digitalWrite(ERROR_LOCAL_LED, errorLocal);
  digitalWrite(PHONE_SIG_OUT_RLY, phoneSigOut);
  digitalWrite(PHONE_SIG_OUT_LED, phoneSigOut);
  digitalWrite(ERROR_REMOTE_RLY, errorRemote);
  digitalWrite(ERROR_REMOTE_LED, errorRemote);
}


void udpSend(char code) {
  Serial.print("Tx: ");
  udp.beginPacket(ipAddresses[pairedID], PORT);
  udp.write(code);
  udp.endPacket();
  Serial.println(code);
}

void udpSendLine(char code) {
  Serial.print("Tx(line): ");
  for (uint8_t lineController = 0; lineController < lineLength; lineController++) {
    if (lineController == localID) continue;
    udp.beginPacket(ipAddresses[lineController], PORT);
    udp.write(code);
    udp.write(localIDChar);
    udp.endPacket();
  }
  Serial.print(code);
  Serial.print(localIDChar);
  Serial.write("\n");
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
  Serial.print("  ping: ");
  Serial.println(millis() - lastTelegraphPacket);
  Serial.print("  'unAcknowledgedTelegraphPackets' = ");
  Serial.println(unAcknowledgedTelegraphPackets);
}

void onReceiveHeartbeat(bool healthy, uint16_t packetSize) {
  char remoteIDchar[packetSize - 1];
  for (uint16_t byte = 0; byte < packetSize - 1; byte++) remoteIDchar[byte] = packetBuffer[byte + 1];
  uint16_t remoteID = atoi(remoteIDchar);
  lastHeartBeatsReceived[remoteID] = millis();
  heartBeatStati[remoteID] = healthy;
}

void throwError(char errorMessage[]) {
  errorLocal = true;
  Serial.print(errorMessage);
  Serial.print("\n  There has been a error @: ");
  Serial.println(millis());
}

void errorLocalAck() {
  if (!errorLocal) return;
  errorLocal = false;
  unAcknowledgedTelegraphPackets = 0;
  Serial.print("Error has been acknowledged @");
  Serial.println(millis());
}

void errorRemoteAck() {
  udpSend('r');
}


void verbosePinMode(pin_size_t pin, PinMode mode) {
  Serial.print(pin);
  Serial.print(" > ");
  Serial.print(mode);
  Serial.print("; ");
  pinMode(pin, mode);
}
