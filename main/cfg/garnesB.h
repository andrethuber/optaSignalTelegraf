
// Config file for garnesB

// Line config:

enum lineControllers {
  garnesA,
  garnesB,
  arnagA,
  arnagB,
  haukelandA,
  haukelandB
};

const uint8_t lineLength = 6;

const char lineName[] = "gvb-main";

IPAddress ipAddresses[] = {
  { 10, 1, 2, 60 },  // garnesA
  { 10, 1, 2, 61 },  // ganresB
  { 10, 2, 2, 60 },  // arnagA
  { 10, 2, 2, 61 },  // arnagB
  { 10, 3, 2, 60 },  // haukelandA
  { 10, 3, 2, 61 },  // haukelandB
};

IPAddress remoteServerIp = { 172, 30, 1, 12 };

// Controller config:

const uint8_t localID = garnesB;
const uint8_t pairedID = arnagA;

byte macAddress[] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x0a };  // 'Ethernet.begin()' wants a MAC address as a argument, but it dosent seem to acually want to use it, as 'Ethernet.MACAddress()' returns something else.

IPAddress localGateway = { 10, 1, 2, 1 };
