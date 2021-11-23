#include <WiFiNINA.h>
// #include "EMailSender.h"

//Wire Library for I2C communication used by the thermal camera
//SCL (clock) is pin A5
//SDA (data) is pin A4
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
//Thermal camera library provided by maufacturer

/* THERMAL CAMERA VARIABLES*/
#define AL_GROUP 8
#define TEMP_MAX 40
#define TEMP_MIN 20
bool binaryMap[AMG88xx_PIXEL_ARRAY_SIZE];

/*FALL SENSOR VARIABLES*/
int pinF = 1;       // connect FSR to A1
int aRead, voltage; // analog read and voltage variables
int thresh = 50;    // (mV) modify this threshhold according to fall test values

/*MICROPONE SENSOR VARIABLES*/
int micVal = 0;       //magnitude of the sound
int micPin = 0;       //connect Microphone to A0
int micTrigger = 600; //threshhold to surpass to trigger sound alert

/*WATER SENSOR VARIABLES*/
int pinW = 2;            // connect FSR to A2
int aRead, voltage;      // analog read and voltage variables
bool alert = false;      // if alert is true, send fall alert to UI
int bottleWeight = 2800; // FSR voltage reading when empty bottle is resting on it.
int thresh = 3200;       // set threshhold (the minimum weight of the bottle before sending alert

//////////////////* WIFI Variables *////////////////////////
char ssid[] = "";            //  your network SSID (name) between the " "
char pass[] = "";            // your network password between the " "
int keyIndex = 0;            // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS; //connection status
WiFiServer server(80);       //server socket
WiFiClient client = server.available();
/////////////////////////////////////////////////////////////

void setup(void) // main setup loop
{
  Serial.begin(9600);
  pinMode(micPin, INPUT); //Init Microphone
  pinMode(pinF, INPUT);
  pinMode(pinW, INPUT);

  //////////////* WIFI CODE *///////////////////
  // while (!Serial)
  //   ;

  // enable_WiFi();
  // connect_WiFi();

  // server.begin();
  // printWifiStatus();
  //////////////////////////////////////////

  //INITIIALIZE THERMAL SENSOR

  //Check for error
  bool status;

  // default settings
  status = amg.begin();
  if (!status)
  {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1)
      ;
  }
}

void loop(void) // main loop
{
  //fallFunction();
  //microphoneFunction();
  //waterFunction();
  //thermalScan();

  // client = server.available();

  if (fallfunction())
  { //Fall Function
    Serial.println("FALL DETECTED");
  }

  if (microphoneFunction())
  { // Microphone
    Serial.println("SOUND DETECTED");
    Serial.print(F("Sound Magnitude: "));
    Serial.println(micVal);
  }

  if (waterFunction())
  { // Water
    Serial.println("LOW WATER");
  }

  //Thermal required commands
  amg.readPixels(pixels);
  createBDM(pixels, binaryMap);
  if (thermalScan(binaryMap))
  { //Thermal
    Serial.println("TEMPERATURE ALERT!");
  }

  delay(50); // we can change this to something that works for everyone
}

/*FALL FUNCTION*/
bool fallFunction(void)
{
  aRead = analogRead(pinF);               // set analog int to read from pin A0
  voltage = map(aRead, 0, 1023, 0, 5000); // voltage (mV) is mapped to analog values

  if (voltage > thresh) // depending on the threshhold, evaluate voltage and print appropriate mesage
  {
    // **use serial statements to test threshholds**
    //      Serial.println("WARNING: HIGH FORCE DETECTED (N): ");
    //      Serial.print(voltage);
    //      Serial.println();
    return true; // send alert
  }
  else
  {
    Serial.println("No incident detected");
    return false;
  }
}
/*END OF FALL FUNCTION*/

/*MICROPHONE FUNCTION*/
bool microphoneFunction(void)
{
  micVal = analogRead(micPin); // read magnitude of sound from microphone
  if (micVal > micTrigger)     //check if magnitude surpasses threshhold
  {
    return true; //send Sound Alert!
  }
  else
    return false; //no warnings
}
/*END OF MICROPHONE FUNCTION*/

/*WATER FUNCTION*/
bool waterFunction(void) // function that returns true if the patient needs a refill
{
  aRead = analogRead(pinW);               // set analog int to read from pin A0
  voltage = map(aRead, 0, 1023, 0, 5000); // voltage (mV) is mapped to analog values

  // **Serial console statements for testing and gathering patient data: bottle weight / acceptable threshhold**
  //     Serial.println("FSR Voltage (mV): ");
  //     Serial.print(voltage);
  //     Serial.println();

  if ((voltage > bottleWeight) && (voltage < thresh)) // if the bottle is on the sensor (patient is not actively using it) and it is almost empty, send warning
  {
    //      Serial.println("WARNING! PATIENT NEEDS REFILL");
    return true;
  }

  return false;
}
/*END OF WATER SECTION*/

//THERMAL SENSOR FUNCTIONS------------------

void createBDM(float data[], bool output[])
{

  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {

    if ((data[i] > TEMP_MAX) || (data[i] < TEMP_MIN))
      output[i] = 1;
    else
      output[i] = 0;
  }
}

/* Array layout
 * 0 - 7        (0)
 * 8 - 15       (1)
 * 16 - 23      (2)
 * 24 - 31      (3)
 * 32 - 39      (4)
 * 40 - 47      (5)
 * 48 - 55      (6)
 * 56 - 63      (7)
 */

int scanCardinals(bool binmap[], int i)
{

  int gCount;

  //Function called on binmap points with TRUE (1) values
  //These are identified as unsafe values
  //When called: set this value to 0 (to prevent double counting) and incriment counter

  if (binmap[i] != 1)
    return -999; //error

  gCount = 1;
  binmap[i] = 0;
  //Scan all cardinal directions

  //Scan left
  //Check left is valid - decrementing should not cause wrap around
  if (((i - 1) % 8) < (i % 8))
  {
    if (binmap[i - 1] == 1)
    {
      gCount += scanCardinals(binmap, i - 1);
    }
  }

  //Scan right
  //Check right is valid - Incrimenting i should not wrap around
  if (((i + 1) % 8) > (i % 8))
  {
    if (binmap[i + 1] == 1)
    {
      gCount += scanCardinals(binmap, i + 1);
    }
  }

  //Scan up
  //Check UP is valid - going up 1 row should be possible
  //If not - on upper array row and cannot check upwards
  if ((i - 8) >= 0)
  {
    if (binmap[i - 8] == 1)
    {
      gCount += scanCardinals(binmap, i - 8);
    }
  }

  //Scan down
  //Check DOWN is valid - going down 1 row should not be outside of the array
  //Skip check if it is, since down is invalid for that case
  if ((i + 8) < AMG88xx_PIXEL_ARRAY_SIZE)
  {
    if (binmap[i + 8] == 1)
    {
      gCount += scanCardinals(binmap, i + 8);
    }
  }

  return gCount;
}

bool thermalScan(bool binmap[])
{

  int groupSize;

  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {
    //Search array for alerts (1) and scan those for groups of 1
    //The scanCardinals will remove the entire group of 1's (set to 0)
    //and count the group size

    if (binmap[i] == 1)
    {
      groupSize = scanCardinals(binmap, i);

      if (printMode)
      {
        Serial.print("Group found: ");
        Serial.print(groupSize);
        Serial.println();
      }

      if (groupSize >= AL_GROUP)
        return true;
    }
  }
  //If this code is reached, the whole array has been scanned and no
  //groups were found of large enough size, so no problems found
  return false;
}

//END OF THERMAL SENSOR FUNCTIONS-----------

////////////////////////* WIFI FUNTION *//////////////////////////
void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void enable_WiFi()
{
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0")
  {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi()
{
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(5000);
  }
}
///////////////////////////////////////////////////////////////
