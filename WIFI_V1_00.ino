/*
""" ############################################################################################ """
""" ############################################################################################ """
""" ESP-32 Dev Kit C V4 connected to 3 SolarMax 4200S via hardwareserial RS485"""
""" Connecting to WIFI network and send MQTT message """
""" I2C connected to temperature/humidity sensor GY-21"""
""" V1_00, 2023-03-25, are """
Changes:
NA
""" ############################################################################################ """
""" ############################################################################################ """
*/

// ##### Hardware: ESP-32 Dev Kit C V4
// With RS485 Hardwareserial to SolarMax 4200S

// ##### Include
#include <WiFi.h> // WIFI
#include <PubSubClient.h> // MQTT
#include <Wire.h> // I2C

// ##### Definitions
#define DEBUGMODE false // Print diagnostic values to serial monitor

// ##### Definitions RS485, Hardwareserial
/*
In general, most pins on the ESP8266 and ESP32 devices can be used by SoftwareSerial,
however each device has a number of pins that have special functions or require careful
handling to prevent undesirable situations, for example they are connected to the on-board
SPI flash memory or they are used to determine boot and programming modes after powerup
or brownouts. These pins are not able to be configured by this library.

It works with RX0 / TX0, but program download doesn't work any more
#define HWSerialRX 3  // RX0 pin, GPIO03
#define HWSerialTX 1  // TX0 pin, GPIO01
*/
#define HWSerialRX 16  // RX2 pin, GPIO16
#define HWSerialTX 17  // TX2 pin, GPIO17
const int BytesSolarMaxToESP = 200;
const int BytesESPToSolarMax = 200;
bool TransmitDataRS485 = false;
bool ReceiveDataRS485MainsOff = false;

// Struct SolarMax 4200S
typedef struct
{
  bool SendDataToMQTTSolarMax;
  char JSONString[BytesSolarMaxToESP];
  int PAC, KHR, KT0, KYR, KMT, KDY, PRL, TKK;
  double dKDY;
} SolarMax;
SolarMax SolarMaxData[3] = {};

// ##### Setup WIFI Network, SSID and password
const char* SSID = "MagentaWLAN-5DHZ"; // Network name
const char* PSK = "12345678901234567890"; // Password
String HostnameWIFI = "ESP32-Node-SolarMax"; // Node name ESP32
WiFiClient espClient; // WiFi network client for MQTT

// ##### Setup MQTT
const char* MQTT_BROKER = "192.168.2.77"; // MQTT broker (server) IP-Address. Raspberry: "192.168.2.51" / MiniPC: "192.168.2.77"
PubSubClient client(espClient); // Creates a partially initialised MQTT client instance. Pass WiFi network client
const char* ClientID = "ESP8266ClientSolarMax"; //  MQTT client ID when connecting to the MQTT broker
const char* TopicSolarMaxData[3] = {"/Groundfloor/SolarMax1/Data", // MQTT topic SolarMax data
                                    "/Groundfloor/SolarMax2/Data",
                                    "/Groundfloor/SolarMax3/Data"};
const char* TopicGroundfloorBasement = "/Groundfloor/Basement/Data"; // MQTT topic groundfloor basement data

// ##### Definitions I2C with temperature/humidity sensor GY-21
// Default pins for I2C ESP32 are: GPIO 21 (SDA), GPIO 22 (SCL)
const int GY21_I2C_ADDR = 0x40; // GY-21 I2C address
const int GY21_READ_TEMP = 0xF3; // GY-21 Read temperature register
const int GY21_READ_HUM = 0xF5; // GY-21 Read humidity register
const int GY21_TIMEOUT = 500; // Timeout receive temperature / humidity
bool TransmitDataI2C = false;

// ##### LED show activity RS485, digital output, GPIO15
#define LEDShowActivityRS485 15

// ##### SolarMax on mains, digital input, GPIO05, GPIO18, GPIO19
// It works with GPIO02, but program download doesn't work any more
#define SOLARMAXMAINS_1 5
#define SOLARMAXMAINS_2 18
#define SOLARMAXMAINS_3 19

// ##### Forward declaration functions
void ReceiveDataRS485(char, char, int);
void SendDataRS485(char, char);
int BuildIntValueFromString(char*, char*);
double ReadTemperatureGY21(const int);
double ReadHumidityGY21(const int);

// ################################################################################
// ##### Setup
void setup()
{  
  // ########## LED on board, digital output
  pinMode(LEDShowActivityRS485, OUTPUT);
  digitalWrite(LEDShowActivityRS485, HIGH); // Switch LED On

  // ##### SolarMax on mains, digital inputs
  pinMode(SOLARMAXMAINS_1, INPUT);
  pinMode(SOLARMAXMAINS_2, INPUT);
  pinMode(SOLARMAXMAINS_3, INPUT);

  // ##### Serial monitor
  if(DEBUGMODE == true)
  {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Startup");
  }

  // ##### Initialize and start Hardwareserial RS485 to SolarMax
  // Die Übertragungsrate beträgt 19200 Bit/s. Das Protokoll ist 8 Datenbits, keine Parität, 1 Stopbit (8N1).

  // Parameter:
  // baud
  // databits, parity, and stop bit count
  // RX pin
  // TX pin
  // invert, true=uses invert line level logic
  // timeout ms
  // Send example: {FB;01;32|64:PAC;KHR;KT0;KYR;KMT;KDY;PRL;TKK|0c27}
  // Receive example: {01;FB;4F|64:PAC=3B2;KHR=D2B2;KT0=EF3E;KYR=1C9;KMT=61;KDY=12;PRL=9;TKK=1E|12DE}
  Serial2.begin(19200, SERIAL_8N1, HWSerialRX, HWSerialTX, false, 512);
  delay(100);

  // If the object did not initialize, then its configuration is invalid
  if (!Serial2)
  {
    if(DEBUGMODE == true)
    {
      Serial.println("Invalid Serial2 pin configuration, check config");
    }
  }

  // ########## Setup and connect to WIFI network
  setup_wifi();

  // ########## MQTT, PubSub client
	client.setServer(MQTT_BROKER, 1883); // Configure broker, IPAdress and port

  // ########## I2C, Temperature and humidity sensor GY-21
  // Default pins for I2C ESP32 are: GPIO 21 (SDA), GPIO 22 (SCL)
  Wire.begin();
  delay(100);
  Wire.beginTransmission(GY21_I2C_ADDR);
  uint8_t error = Wire.endTransmission(); // Returns byte, 0=success
  if(DEBUGMODE == true)
  {
    if (error != 0 && error != 7)
    {
      Serial.print("I2C ERROR CODE GY21:");
      Serial.println(error);
    }
    else
    {
      Serial.println("I2C connected to GY21");
    }
  }

} // void setup()

// ################################################################################
// ##### Loop
void loop()
{
  // ##### Local variables
  static long Timer = 0;
  static long Delaytime = 20000; // 20sec
  static char SolarMaxAddressLow = 48; // dez 48, Char '0'
  static char SolarMaxAddressHigh = 48; // dez 48, Char '0'
  static int DigitalInputMainsOperation = LOW; // LOW 0x0
  int StatusDigitalInput;
  static int StatusDigitalInputMainsOperation_1 = HIGH; // HIGH 0x1
  static int StatusDigitalInputMainsOperation_2 = HIGH;
  static int StatusDigitalInputMainsOperation_3 = HIGH;
  double Temperature;
  double Humidity;
  char JSONStringGroundfloorBasement[200];
  int i;

  // ########## MQTT check connection to broker
	if (!client.connected()) // Returns true when connected to the broker
  {
		connect(); // Call function Connect to MQTT broker (server)
	}
	client.loop();

  // ########## Status digital input, set to false between two RS485 send activities
  StatusDigitalInput = digitalRead(SOLARMAXMAINS_1);
  if(StatusDigitalInput == LOW)
  {
    StatusDigitalInputMainsOperation_1 = LOW;
  }

  StatusDigitalInput = digitalRead(SOLARMAXMAINS_2);
  if(StatusDigitalInput == LOW)
  {
    StatusDigitalInputMainsOperation_2 = LOW;
  }

  StatusDigitalInput = digitalRead(SOLARMAXMAINS_3);
  if(StatusDigitalInput == LOW)
  {
    StatusDigitalInputMainsOperation_3 = LOW;
  }

  // ########## Timer send data RS485
  if (millis() > Delaytime + Timer )
  {
    // Actualize Timer
    Timer = millis();

    // ##### Start send data RS485 SolarMax
    TransmitDataRS485 = true;

    // ##### Calculate SolarMax address dez 49..51, Char '1'..'3'
    SolarMaxAddressLow +=1; // Transfer address low +1
    if(SolarMaxAddressLow >= 52) // >=dez 52, Char 4
    {
      SolarMaxAddressLow = 49; // Transfer address low dez 49, Char '1'
    }

    // ##### Transfer digital input SolarMax on mains operation
    DigitalInputMainsOperation = 0;
    if(SolarMaxAddressLow == 49) // SolarMax adress 1
    {
      DigitalInputMainsOperation = StatusDigitalInputMainsOperation_1;
      StatusDigitalInputMainsOperation_1 = HIGH;
      if(DigitalInputMainsOperation == LOW)
      {
        TransmitDataRS485 = false;
        ReceiveDataRS485MainsOff = true;
      }
    }
   
    if(SolarMaxAddressLow == 50) // SolarMax adress 2
    {
      DigitalInputMainsOperation = StatusDigitalInputMainsOperation_2;
      StatusDigitalInputMainsOperation_2 = HIGH;
      if(DigitalInputMainsOperation == LOW)
      {
        TransmitDataRS485 = false;
        ReceiveDataRS485MainsOff = true;
      }
    }

    if(SolarMaxAddressLow == 51) // SolarMax adress 3
    {
      DigitalInputMainsOperation = StatusDigitalInputMainsOperation_3;
      StatusDigitalInputMainsOperation_3 = HIGH;
      if(DigitalInputMainsOperation == LOW)
      {
        TransmitDataRS485 = false;
        ReceiveDataRS485MainsOff = true;
      }
    }

    if(DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.print("DigitalInput Status, Adress ");
      Serial.print(SolarMaxAddressLow);
      Serial.print(": ");
      Serial.println(DigitalInputMainsOperation);
    }

    // ##### Start send data I2C GY-21
    if(SolarMaxAddressLow == 49)
      TransmitDataI2C = true;
  } // if (millis() > Delaytime + Timer )

  // ########## RS485 send / receive data
  SendDataRS485(SolarMaxAddressLow, SolarMaxAddressHigh); // Call function send data RS485
  ReceiveDataRS485(SolarMaxAddressLow, SolarMaxAddressHigh, DigitalInputMainsOperation); // Call function receive data RS485

  // ########## I2C, Temperature and humidity sensor GY-21
  if(TransmitDataI2C == true)
  {
    TransmitDataI2C = false;

    // ##### Temperature
    Temperature = ReadTemperatureGY21(GY21_I2C_ADDR);
    if(DEBUGMODE == true)
    {
      Serial.print("Temperature: ");
      Serial.print(Temperature);
      Serial.println("*C");
    }

    // ##### Humidity
    Humidity = ReadHumidityGY21(GY21_I2C_ADDR);
    if(DEBUGMODE == true)
    {
      Serial.print("Humidity: ");
      Serial.print(Humidity);
      Serial.println("%");
    }

    // ##### MQTT publish message temperature and humidity
    sprintf(JSONStringGroundfloorBasement, "{\"Temperature\":%.1f,\"Humidity\":%.1f}", Temperature, Humidity);
    client.publish(TopicGroundfloorBasement, JSONStringGroundfloorBasement);
  }

  // ########## MQTT publish message SolarMax data
  for(i=0; i<3; i++)
  {
    if(SolarMaxData[i].SendDataToMQTTSolarMax == true)
    {
      SolarMaxData[i].SendDataToMQTTSolarMax = false;
      client.publish(TopicSolarMaxData[i], SolarMaxData[i].JSONString);
    }
  }

} // void loop()

// ################################################################################
// ##### RS485 receive data from SolarMax
// {01;FB;4E|64:PAC=510;KHR=D18E;KT0=EE1C;KYR=A7;KMT=5A;KDY=56;PRL=C;TKK=23|12AA}
void ReceiveDataRS485(char iAddressLow, char iAdressHigh, int iDigitalInputMainsOperation)
{
  // ##### Local variables
  int i;
  bool StringOK;
  char receivedChar;
  char msg[50] = "";
  char Mains[10] = "";
  static int activeCharacter = 0;
  static char t[BytesSolarMaxToESP]= ""; // Empty array where to put the received data from RS485
  int ArrayIndex;

  // ########## Is Serial2 available
  if (Serial2.available() > 0)
  {
    // Copy received character to char array
    receivedChar = Serial2.read(); // Read received byte
    if(activeCharacter < BytesSolarMaxToESP)
    {
      t[activeCharacter] = receivedChar;
      activeCharacter++;      
    }
    else
    {
      activeCharacter = 0; // Clear character counter
      // Clear received data
      for(i=0; i<BytesSolarMaxToESP; i++)
      {
        t[i] = '\0'; // Clear received data
      }
    }

    if(receivedChar == '}') // ETX, End of text, dez 125, Char }
    {
      activeCharacter++;
      t[activeCharacter] = '\0'; // Add Null termination character
      activeCharacter = 0; // Clear character counter when received end characters

      if (DEBUGMODE == true)
      {
        Serial.println(" ");
        Serial.println("Receive RS485 Char:");
        Serial.print(t);
        Serial.println(" ");
      }

      // ##### Charactercheck
      StringOK = false;
      if(t[0] == '{'
      && t[1] == iAdressHigh 
      && t[2] == iAddressLow
      )
      {
        StringOK = true;
      } // Charactercheck

      // ##### Charactercheck ok, evaluate received values
      if(StringOK == true)
      {

        // ##### Calculate SolarMax address
        if(iAddressLow == 49) // dez 49, Char '1'
        {
          ArrayIndex = 0;
        }
        else if (iAddressLow == 50) // dez 50, Char '2'
        {
          ArrayIndex = 1;
        }
        else if (iAddressLow == 51) // dez 51, Char '3'
        {
          ArrayIndex = 2;
        }
        else
        {
          ArrayIndex = 0;
        }

        // ##### Bulid JSON string from received values
        // SolarMax receive: {01;FB;4E|64:PAC=510;KHR=D18E;KT0=EE1C;KYR=A7;KMT=5A;KDY=56;PRL=C;TKK=23|12AA}
        /*
        PAC AC-Leistung, Auflösung 0,5 (Wert / 2)
        KHR Betriebsstunden, Auflösung 1.0 (Wert / 1)
        KT0 Energie Total, Auflösung 1.0 (Wert / 1)
        KYR Energie Jahr, Auflösung 1.0 (Wert / 1)
        KMT Energie Monat, Auflösung 1.0 (Wert / 1)
        KDY Energie Tag, Auflösung 0,1 (Wert / 10)
        PRL Relative Leistung in %, Auflösung 1.0 (Wert / 1)
        TKK Temperatur Leistungsteil 1, Auflösung 1.0 (Wert / 1)
        */
        // Build PAC value as an integer, hex string to dec
        strcpy(msg, "PAC=");
        SolarMaxData[ArrayIndex].PAC = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        SolarMaxData[ArrayIndex].PAC = SolarMaxData[ArrayIndex].PAC / 2;
        // Build KHR value as an integer, hex string to dec
        strcpy(msg, "KHR=");
        SolarMaxData[ArrayIndex].KHR = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        // Build KT0 value as an integer, hex string to dec
        strcpy(msg, "KT0=");
        SolarMaxData[ArrayIndex].KT0 = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        // Build KYR value as an integer, hex string to dec
        strcpy(msg, "KYR=");
        SolarMaxData[ArrayIndex].KYR = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        // Build KMT value as an integer, hex string to dec
        strcpy(msg, "KMT=");
        SolarMaxData[ArrayIndex].KMT = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        // Build KDY value as an integer, hex string to dec
        strcpy(msg, "KDY=");
        SolarMaxData[ArrayIndex].KDY = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        SolarMaxData[ArrayIndex].dKDY = double(SolarMaxData[ArrayIndex].KDY) / 10.0;
        // Build PRL value as an integer, hex string to dec
        strcpy(msg, "PRL=");
        SolarMaxData[ArrayIndex].PRL = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find
        // Build TKK value as an integer, hex string to dec
        strcpy(msg, "TKK=");
        SolarMaxData[ArrayIndex].TKK = BuildIntValueFromString(t, msg); // t = Complete string, msg = String to find

        // Digital input, build string ""on"" or ""off""
        if(iDigitalInputMainsOperation == HIGH)
        {
          sprintf(Mains, "\"on\"");
        }
        else
        {
          sprintf(Mains, "\"off\"");
        }

        // ##### Build JSON-String
        // {"PAC":615,"KHR":53865,"KT0":61211,"KYR":421,"KMT":62,"KDY":1.2,"PRL":12,"TKK":26,"MAINS":"on"}
        for(i=0; i<BytesSolarMaxToESP; i++)
        {
          SolarMaxData[ArrayIndex].JSONString[i] = '\0'; // Clear JSON string
        }
        sprintf(SolarMaxData[ArrayIndex].JSONString, "{\"PAC\":%i,\"KHR\":%i,\"KT0\":%i,\"KYR\":%i,\"KMT\":%i,\"KDY\":%.1f,\"PRL\":%i,\"TKK\":%i,\"MAINS\":%s}",
                                                      SolarMaxData[ArrayIndex].PAC,
                                                      SolarMaxData[ArrayIndex].KHR,
                                                      SolarMaxData[ArrayIndex].KT0,
                                                      SolarMaxData[ArrayIndex].KYR,
                                                      SolarMaxData[ArrayIndex].KMT,
                                                      SolarMaxData[ArrayIndex].dKDY,
                                                      SolarMaxData[ArrayIndex].PRL,
                                                      SolarMaxData[ArrayIndex].TKK,
                                                      Mains);
        SolarMaxData[ArrayIndex].SendDataToMQTTSolarMax = true;

        if (DEBUGMODE == true)
        {
          Serial.println(" ");
          Serial.println("Send JSON-String:");
          Serial.println(SolarMaxData[ArrayIndex].JSONString);
          Serial.println("########################################################################");
        }

      } // if(StringOK == true)

      // ##### Clear received data
      for(i=0; i<BytesSolarMaxToESP; i++)
      {
        t[i] = '\0'; // Clear received data
      }
    } // if(receivedChar == '}')

  } // if (Serial2.available() > 0)


  // ########## SolarMax mains == off
  if(ReceiveDataRS485MainsOff == true)
  {
    ReceiveDataRS485MainsOff = false;

    // ##### Calculate SolarMax address
    if(iAddressLow == 49) // dez 49, Char '1'
    {
      ArrayIndex = 0;
    }
    else if (iAddressLow == 50) // dez 50, Char '2'
    {
      ArrayIndex = 1;
    }
    else if (iAddressLow == 51) // dez 51, Char '3'
    {
      ArrayIndex = 2;
    }
    else
    {
      ArrayIndex = 0;
    }

    // ##### Send JSON-String
    // PAC = 0, PRL = 0, TKK = 0, Mains = off
    // {"PAC":615,"KHR":53865,"KT0":61211,"KYR":421,"KMT":62,"KDY":1.2,"PRL":12,"TKK":26,"MAINS":"on"}
    SolarMaxData[ArrayIndex].PAC = 0;
    SolarMaxData[ArrayIndex].PRL = 0;
    SolarMaxData[ArrayIndex].TKK = 0;
    sprintf(Mains, "\"off\"");

    for(i=0; i<BytesSolarMaxToESP; i++)
    {
      SolarMaxData[ArrayIndex].JSONString[i] = '\0'; // Clear JSON string
    }
    sprintf(SolarMaxData[ArrayIndex].JSONString, "{\"PAC\":%i,\"KHR\":%i,\"KT0\":%i,\"KYR\":%i,\"KMT\":%i,\"KDY\":%.1f,\"PRL\":%i,\"TKK\":%i,\"MAINS\":%s}",
                                                  SolarMaxData[ArrayIndex].PAC,
                                                  SolarMaxData[ArrayIndex].KHR,
                                                  SolarMaxData[ArrayIndex].KT0,
                                                  SolarMaxData[ArrayIndex].KYR,
                                                  SolarMaxData[ArrayIndex].KMT,
                                                  SolarMaxData[ArrayIndex].dKDY,
                                                  SolarMaxData[ArrayIndex].PRL,
                                                  SolarMaxData[ArrayIndex].TKK,
                                                  Mains);
    SolarMaxData[ArrayIndex].SendDataToMQTTSolarMax = true;

    if (DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.println("Send JSON-String:");
      Serial.println(SolarMaxData[ArrayIndex].JSONString);
      Serial.println("########################################################################");
    }
  } // if(ReceiveDataRS485MainsOff == true)

} // void ReceiveDataRS485()

// ################################################################################
// ##### RS485 send data to SolarMax
void SendDataRS485(char iAddressLow, char iAdressHigh)
{
  /*
  Die Übertragungsrate beträgt 19200 Bit/s. Das Protokoll ist 8 Datenbits, keine Parität, 1 Stopbit (8N1).

  STX Src-Adr FS Dest-Adr FS Length FRS Port US Data FRS Crc ETX oder ETB
  Senden:     {FB;2A;<Length>|64:TYP;SWV;UDC|<Crc>} // Data: Typ, Softwareversion, DC-Spannung
  Empfangen:  {2A;FB;<Length>|64:TYP=7D0;SWV=28;UDC=180|<Crc>}
  
  Beispiel senden:    {FB;05;4E|64:E1D;E11;E1h;E1m;E1M;E2D;E21;E2h;E2m;E2M;E3D;E31;E3h;E3m;E3M|1270}
  Beispiel empfangen: {05;FB;7C|64:E1D=16;E11=8002;E1h=12;E1m=39;E1M=3;E2D=1C;E21=8004;E2h=E;E2m=21;E2M=3;E3D=6;E31=8004;E3h=10;E3m=23;E3M=4|1C4E}

  Src-Adr 0xFB    = dez 251   2 ASCII-Zeichen lang = RS485 master address 251 (ESP8266)
  Dest-Adr 0x05   = dez 5     2 ASCII-Zeichen lang = RS485 slave address 5 (SolarMax)
  Length  0x4E    = dec 78    2 ASCII-Zeichen lang = Gesamtzeichenlänge des gesendeten Strings = 78 ASCII-Zeichen
  Length  0x7C    = dec 124   2 ASCII-Zeichen lang = Gesamtzeichenlänge des empfangenen Strings = 124 ASCII-Zeichen
  Port    0x64    = dec 100   2 ASCII-Zeichen lang = Command Nutzdaten abfragen
  Port    0xC8    = dez 200   2 ASCII-Zeichen lang = Einstellung oder Befehl
  Port    0x3E8   = dez 1000  2 ASCII-Zeichen lang = Meldungen von der Schnittstelle
  CRC     0x1270  = dec 4720  4 ASCII-Zeichen lang = CRC Summe der ASCII-Werte aller Zeichen von Adress bis und mit dem FRS vor Crc, 4-Stellig, in Hex value, führend '0'
  CRC     0x1C4E  = dec 7246  4 ASCII-Zeichen lang = CRC Summe der ASCII-Werte aller Zeichen von Adress bis und mit dem FRS vor Crc, 4-Stellig, in Hex value, führend '0'

  Data (Auszug):
  TYP Typ
  PAC AC-Leistung, Auflösung 0,5 (Wert / 2)
  KHR Betriebsstunden, Auflösung 1.0 (Wert / 1)
  KT0 Energie Total, Auflösung 1.0 (Wert / 1)
  KYR Energie Jahr, Auflösung 1.0 (Wert / 1)
  KMT Energie Monat, Auflösung 1.0 (Wert / 1)
  KDY Energie Tag, Auflösung 0,1 (Wert / 10)
  PRL Relative Leistung in %, Auflösung 1.0 (Wert / 1)
  TKK Temperatur Leistungsteil 1, Auflösung 1.0 (Wert / 1)

  STX, Start of text, dez 123, Char {
  FS, Field separator, dez 59, Char ;
  FRS, Frame Separator, dez 124, Char |
  US, Union Separator, dez 58, Char :
  ETX, End of text, dez 125, Char }

  Examples:
  TYP
  {FB;01;16|64:TYP|045f}
  {01;FB;1B|64:TYP=4E3E|0599}
  PAC
  {FB;01;16|64:PAC|0436}
  {01;FB;1A|64:PAC=EBA|0546}
  KHR
  {FB;01;16|64:KHR|0447}
  {01;FB;1B|64:KHR=D18B|057F}
  KT0
  {FB;01;16|64:KT0|0431}
  {01;FB;1B|64:KT0=EE16|056B}
  KDY
  {FB;01;16|64:KDY|044a}
  {01;FB;19|64:KDY=2B|04FE}
  PRL
  {FB;01;16|64:PRL|0450}
  {01;FB;19|64:PRL=37|04FA}
  TKK
  {FB;01;16|64:TKK|044c}
  {01;FB;19|64:TKK=2C|04F7}
  PAC;KHR;KT0;KYR;KMT;KDY;PRL;TKK
  {FB;01;32|64:PAC;KHR;KT0;KYR;KMT;KDY;PRL;TKK|0c27}
  {01;FB;4F|64:PAC=81E;KHR=D18C;KT0=EE1A;KYR=A5;KMT=58;KDY=42;PRL=14;TKK=2C|12E1}
  */

  // ##### Local variables
  char t[BytesESPToSolarMax]= ""; // Empty array where to put the data
  uint8_t AmountOfBytesWriteRS485 = 0;
  int CRCSumme = 0;
  int i;
  char msg[50];
  char zeichen;

  // ##### Is transmit data requested ?
  if(TransmitDataRS485 == true)
  {
    TransmitDataRS485 = false;

    // ##### LED on board on
    digitalWrite(LEDShowActivityRS485, HIGH); // Switch LED On

    // ##### Set write data array
    t[0] = '{'; // STX, Start of text, dez 123, Char {
  
    t[1] = 'F'; // Src-Adr 0xFB = dez 251, 2 ASCII-Zeichen lang = RS485 master address 251 (ESP8266)
    t[2] = 'B';
    t[3] = ';'; // FS, Field separator, dez 59, Char ;

    t[4] = iAdressHigh; // '0'; // Dest-Adr 0x01 = dez 1, 2 ASCII-Zeichen lang = RS485 slave address 1 (SolarMax)
    t[5] = iAddressLow; // '1';
    t[6] = ';'; // FS, Field separator, dez 59, Char ;

    t[7] = '3'; // Length, 0x32 = dez 50, Länge des gesamt gesendeten Strings = 50 ASCII-Zeichen
    t[8] = '2';
    t[9] = '|'; // FRS, Frame Separator, dez 124, Char |
    
    t[10] = '6'; // Port, 0x64 = dez 100 = Nutzdaten (Daten abfragen)
    t[11] = '4';
    t[12] = 58; // US, Union Separator, dez 58, Char :
  
    /*
    Data:
    PAC AC-Leistung, Auflösung 0,5 (Wert / 2)
    KHR Betriebsstunden, Auflösung 1.0 (Wert / 1)
    KT0 Energie Total, Auflösung 1.0 (Wert / 1)
    KYR Energie Jahr, Auflösung 1.0 (Wert / 1)
    KMT Energie Monat, Auflösung 1.0 (Wert / 1)
    KDY Energie Tag, Auflösung 0,1 (Wert / 10)
    PRL Relative Leistung in %, Auflösung 1.0 (Wert / 1)
    TKK Temperatur Leistungsteil 1, Auflösung 1.0 (Wert / 1)
    */
    t[13] = 'P'; // Data Byte 0, Char T
    t[14] = 'A'; // Data Byte 1, Char Y
    t[15] = 'C'; // Data Byte 2, Char P
    t[16] = ';'; // FS, Field separator, dez 59, Char ;

    t[17] = 'K'; // Data Byte 0, Char T
    t[18] = 'H'; // Data Byte 1, Char Y
    t[19] = 'R'; // Data Byte 2, Char P
    t[20] = ';'; // FS, Field separator, dez 59, Char ;

    t[21] = 'K'; // Data Byte 0, Char T
    t[22] = 'T'; // Data Byte 1, Char Y
    t[23] = '0'; // Data Byte 2, Char P
    t[24] = ';'; // FS, Field separator, dez 59, Char ;

    t[25] = 'K'; // Data Byte 0, Char T
    t[26] = 'Y'; // Data Byte 1, Char Y
    t[27] = 'R'; // Data Byte 2, Char P
    t[28] = ';'; // FS, Field separator, dez 59, Char ;

    t[29] = 'K'; // Data Byte 0, Char T
    t[30] = 'M'; // Data Byte 1, Char Y
    t[31] = 'T'; // Data Byte 2, Char P
    t[32] = ';'; // FS, Field separator, dez 59, Char ;

    t[33] = 'K'; // Data Byte 0, Char T
    t[34] = 'D'; // Data Byte 1, Char Y
    t[35] = 'Y'; // Data Byte 2, Char P
    t[36] = ';'; // FS, Field separator, dez 59, Char ;

    t[37] = 'P'; // Data Byte 0, Char T
    t[38] = 'R'; // Data Byte 1, Char Y
    t[39] = 'L'; // Data Byte 2, Char P
    t[40] = ';'; // FS, Field separator, dez 59, Char ;

    t[41] = 'T'; // Data Byte 0, Char T
    t[42] = 'K'; // Data Byte 1, Char Y
    t[43] = 'K'; // Data Byte 2, Char P

    t[44] = '|'; // FRS, Frame Separator, dez 124, Char |
  
    // Calc CRC, 4-Stellig, ohne STX
    if (DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.println("Transmit RS485 CRC: ");
    }
    for(i=1; i<=44; i++)
    {
      CRCSumme = CRCSumme + int(t[i]);
      if (DEBUGMODE == true)
      {
        Serial.print(CRCSumme);
        Serial.print(" ");
      }
    }

    if (DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.println("Transmit RS485 CRC INT: ");
      sprintf(msg, "%04i ", CRCSumme);
      Serial.print(msg);
      Serial.println(" ");
    }
    
    sprintf(msg, "%04x ", CRCSumme);
    t[45] = msg[0]; // CRC 0
    t[46] = msg[1]; // CRC 1
    t[47] = msg[2]; // CRC 2
    t[48] = msg[3]; // CRC 3

    t[49] = '}'; // ETX, End of text, dez 125, Char }

    if (DEBUGMODE == true)
    {
      Serial.println("Transmit RS485 CRC HEX: ");
      Serial.print(msg);
      Serial.println(" ");
    }

    // ##### RS485 send data
    AmountOfBytesWriteRS485 = 50;
    Serial2.write(t, AmountOfBytesWriteRS485);
    delay(50);

    // ##### Debug string on serial monitor
    if (DEBUGMODE == true)
    {
      Serial.println("Transmit RS485 Char:");
      for(i=0; i<AmountOfBytesWriteRS485; i++)
      {
        zeichen = char(t[i]);
        Serial.print(zeichen);
      }
      Serial.println(" ");

      Serial.println("Transmit RS485 INT:");
      for(i=0; i<AmountOfBytesWriteRS485; i++)
      {
        sprintf(msg, "%i ", t[i]);
        Serial.print(msg);
      }
      Serial.println(" ");

      Serial.println("Transmit RS485 HEX:");
      for(i=0; i<AmountOfBytesWriteRS485; i++)
      {
        sprintf(msg, "%x ", t[i]);
        Serial.print(msg);
      }
      Serial.println(" ");
    }

    // ##### LED on board off
    digitalWrite(LEDShowActivityRS485, LOW); // Switch LED Off

  } // if(TransmitDataRS485 == true)
} // void SendDataRS485()

// ################################################################################
// ##### Build int value from string
// In characters 4..7 after the first letter of the found string are the hex values
// SolarMax receive: {01;FB;4E|64:PAC=510;KHR=D18E;KT0=EE1C;KYR=A7;KMT=5A;KDY=56;PRL=C;TKK=23|12AA}
// t = Complete string
// msg = String to find
// Factor = result / Factor
// Return int value
int BuildIntValueFromString(char* t, char* msg)
{
  int i;
  int j;
  int ret = 0;

  // ###### Build value as an integer, hex string to dec / 2
  char *ptr = strstr(t, msg); // Search substring
  if (ptr != NULL) // Substring found
  {
    j = 0;
    for(i=4; i<=7; i++) // Go to character 4..7 for the hex value
    {
      if(ptr[i] != ';')
      {
        msg[j] = ptr[i]; // Build hex string
        j++;
      }
    }
    msg[j] = '\0'; // Add Null termination character to hex string
    ret = strtol(msg, 0, 16); // HEX string to int
  } // if (ptr != NULL) // Substring found

  return ret;
}

// ########## Connect to WIFI network
void setup_wifi()
{
  // Print: Connecting to WIFI networkname
  if (DEBUGMODE == true)
  {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);
  }

  // Connect to WIFI network
  // Station (STA) mode is used to get the ESP module connected
  // to a Wi-Fi network established by an access point.
  WiFi.mode(WIFI_STA); // Mode = WIFI station (STA)
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HostnameWIFI.c_str()); //define hostname
	WiFi.begin(SSID, PSK); // Connect to WIFI network

  if (DEBUGMODE == true)
  {
    WiFi.printDiag(Serial); // Print WIFI diagnostic information
  }
  

  // Wait until WIFI connection is etablished
  /*
  WiFi.status:
  0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
  1 : WL_NO_SSID_AVAILin case configured SSID cannot be reached
  2 : WL_SCAN_COMPLETED
  3 : WL_CONNECTED after successful connection is established
  4 : WL_CONNECT_FAILED if connection failed
  5 : WL_CONNECTION_LOST
  6 : WL_CONNECT_WRONG_PASSWORD if password is incorrect
  7 : WL_DISCONNECTED if module is not configured in station mode
  */
  int WIFIStatus = 7;
  char outStr[100];
	while (WIFIStatus != WL_CONNECTED)
  {
    WIFIStatus = WiFi.status();
    if (DEBUGMODE == true)
    {
      sprintf(outStr, "WIFI Status during connecting to network: %d", WIFIStatus);
      Serial.println(outStr);
    }
    delay(500);
	}

  // WIFI is connected, print IP-Address
  if (DEBUGMODE == true)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

// ########## Connect to MQTT broker (server)
/*
-4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
-3 : MQTT_CONNECTION_LOST - the network connection was broken
-2 : MQTT_CONNECT_FAILED - the network connection failed
-1 : MQTT_DISCONNECTED - the client is disconnected cleanly
0 : MQTT_CONNECTED - the client is connected
1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect

-2 means it's failing to create a network connection to the broker.
*/
void connect()
{
	while (!client.connected())
  {
    if (DEBUGMODE == true)
    {
		  Serial.println("Connecting to MQTT broker...");
    }
		if (!client.connect(ClientID))
    {
      if (DEBUGMODE == true)
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" retrying in 5 seconds");
      }
			delay(5000);
		}
	}
}

// ########## I2C, read temperature sensor GY-21
double ReadTemperatureGY21(const int addr)
{
  static double LastTemperature = 0.0;
  int i;
  uint8_t ErrorEndTransmission;
  bool ErrorI2C;

  // ##### Set register for read temperature
  Wire.beginTransmission(addr);
  Wire.write(GY21_READ_TEMP);
  ErrorEndTransmission = Wire.endTransmission(); // Returns byte, 0=success
  ErrorI2C = false;
  if(ErrorEndTransmission != 0 && ErrorEndTransmission != 7)
  {
    ErrorI2C = true;
  }

  if(ErrorI2C == true)
  {
    if(DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.print("I2C ERROR CODE: ");
      Serial.println(ErrorEndTransmission);
    }
    return LastTemperature;
  }

  // ##### Read temperature, 2 byte
  i = 0;
  while (2 != Wire.requestFrom(addr, 2))
  {
    i++;
    if(i>GY21_TIMEOUT)  // Timeout read value
    {
      if (DEBUGMODE == true)
      {
        Serial.println(" ");
        Serial.print("GY21 timeout value read temperature I2C: ");
        Serial.println(i);
      }
      return LastTemperature;
    }
  }
  LastTemperature = ((((Wire.read() << 8) | (Wire.read() & 0b11111100)) * 175.72f) / 65536.0f) -46.85f;
  if(DEBUGMODE == true)
  {
    Serial.println(" ");
    Serial.print("GY21 timeout value read temperature I2C: ");
    Serial.println(i);
  }

  return LastTemperature;
}

// ########## I2C, read humidity sensor GY-21
double ReadHumidityGY21(const int addr)
{
  static double LastHumidity = 0.0;
  int i;
  uint8_t ErrorEndTransmission;
  bool ErrorI2C;

  // ##### Set register for read humidity
  Wire.beginTransmission(addr);
  Wire.write(GY21_READ_HUM);
  ErrorEndTransmission = Wire.endTransmission(); // Returns byte, 0=success
  ErrorI2C = false;
  if(ErrorEndTransmission != 0 && ErrorEndTransmission != 7)
  {
    ErrorI2C = true;
  }

  if(ErrorI2C == true)
  {
    if(DEBUGMODE == true)
    {
      Serial.println(" ");
      Serial.print("I2C ERROR CODE: ");
      Serial.println(ErrorEndTransmission);
    }
    return LastHumidity;
  }

  // ##### Read humidity, 2 byte
  i = 0;
  while (2 != Wire.requestFrom(addr, 2))
  {
    i++;
    if(i>GY21_TIMEOUT)  // Timeout read value
    {
      if (DEBUGMODE == true)
      {
        Serial.println(" ");
        Serial.print("GY21 timeout value read humidity I2C: ");
        Serial.println(i);
      }
      return LastHumidity;
    }
  }
  LastHumidity = ((((Wire.read() << 8) | (Wire.read() & 0b11111100)) * 125.0f) / 65536.0f) -6.0f;
  if(DEBUGMODE == true)
  {
    Serial.println(" ");
    Serial.print("GY21 timeout value read humidity I2C: ");
    Serial.println(i);
  }

  return LastHumidity;
}



