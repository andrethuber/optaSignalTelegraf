
// Config file for null


enum lineControllers {
  null
};

const uint8_t localID = null;
const uint8_t pairedID = null;

const uint8_t lineLength = 0;

const char lineName[] = "null";

IPAddress ipAddresses[] = {
  { 0, 0, 0, 0 }  // null
};

byte macAddress[] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x00 };  // 'Ethernet.begin()' wants a MAC address as a argument, but it dosent seem to acually want to use it, as 'Ethernet.MACAddress()' returns something else.

IPAddress localGateway = { 0, 0, 0, 0 };
IPAddress remoteServerIp = { 0, 0, 0, 0 };

