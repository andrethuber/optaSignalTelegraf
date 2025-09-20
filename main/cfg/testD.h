
// Config file for testD

// Line config:

enum lineControllers {
  testC,
  testD
};

const uint8_t lineLength = 2;

const char lineName[] = "test1";

IPAddress ipAddresses[] = {
  { 192, 168, 1, 50 },  // testC
  { 192, 168, 1, 51 },  // testD
};

IPAddress remoteServerIp = { 172, 30, 1, 12 };

// Controller config:

const uint8_t localID = testD;
const uint8_t pairedID = testC;

byte macAddress[] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x0a };  // 'Ethernet.begin()' wants a MAC address as a argument, but it dosent seem to acually want to use it, as 'Ethernet.MACAddress()' returns something else.

IPAddress localGateway = { 192, 168, 1, 1 };
