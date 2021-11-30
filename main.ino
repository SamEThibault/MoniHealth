#include <WiFiNINA.h>

//Wire Library for I2C communication used by the thermal camera
//SCL (clock) is pin A5
//SDA (data) is pin A4
#include <Wire.h>

/*FALL SENSOR VARIABLES*/
int pinF = 1;         // connect FSR to A1
int aReadF, voltageF; // analog read and voltage variables
int threshF = 50;     // (mV) modify this threshhold according to fall test values

/*MICROPONE SENSOR VARIABLES*/
int micVal = 0;       //magnitude of the sound
int micPin = 0;       //connect Microphone to A0
int micTrigger = 600; //threshhold to surpass to trigger sound alert

/*WATER SENSOR VARIABLES*/
int pinW = 2;            // connect FSR to A2
int aReadW, voltageW;    // analog read and voltage variables
bool alert = false;      // if alert is true, send fall alert to UI
int bottleWeight = 2800; // FSR voltage reading when empty bottle is resting on it.
int threshW = 3200;      // set threshhold (the minimum weight of the bottle before sending alert

/*THERMAL SENSOR VARIABLES*/
int thermalPin = 2;

//////////////////* WIFI Variables *////////////////////////
char ssid[] = ""; //  your network SSID (name) between the " "
char pass[] = "";   // your network password between the " "
int keyIndex = 0;             // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;  //connection status
WiFiServer server(80);        //server socket
WiFiClient client = server.available();
/////////////////////////////////////////////////////////////

void setup(void) // main setup loop
{
  Serial.begin(9600);
  pinMode(micPin, INPUT); //Init Microphone
  pinMode(pinF, INPUT);
  pinMode(pinW, INPUT);
  pinMode(thermalPin, INPUT);

  //////////////* WIFI CODE *///////////////////
  while (!Serial)
    ;

  enable_WiFi();
  connect_WiFi();

  server.begin();
  printWifiStatus();
  //////////////////////////////////////////

  delay(100);
}

void loop(void) // main loop
{
  client = server.available();
  if (client)
  {
    String message = "";
    if (fallFunction() && microphoneFunction()) // if loud noise and pressure on FSR at the same time
    {                                           //Fall Function
      Serial.println("FALL DETECTED");
      message += "Fall Dectected";
    }

     if (microphoneFunction())
     { // Microphone
//        Serial.println("DETECTED sound", message);
//       Serial.print(F("Sound Magnitude: "));
//       Serial.println(micVal);
        message += ", Sound Detected";
     }

    if (waterFunction())
    { // Water
      Serial.println("DETECTED WATER");
      message += ", Low Water Detected";
    }

    if (digitalRead(thermalPin) == HIGH)
    {                                         //Thermal
      Serial.println("DETECTED temperature"); // - Matthew
      message += ", Temperature Alert Detected";
    }
    
    if(message == ""){
      message += "No incidents";
    }
    
    printWEB(message);
  }
  delay(300); // we can change this to something that works for everyone
}

/*FALL FUNCTION*/
bool fallFunction(void)
{
  aReadF = analogRead(pinF);                // set analog int to read from pin A0
  voltageF = map(aReadF, 0, 1023, 0, 5000); // voltage (mV) is mapped to analog values

  if (voltageF > threshF) // depending on the threshhold, evaluate voltage and print appropriate mesage
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
  aReadW = analogRead(pinW);                // set analog int to read from pin A0
  voltageW = map(aReadW, 0, 1023, 0, 5000); // voltage (mV) is mapped to analog values

  // **Serial console statements for testing and gathering patient data: bottle weight / acceptable threshhold**
  //     Serial.println("FSR Voltage (mV): ");
  //     Serial.print(voltage);
  //     Serial.println();

  if ((voltageW > bottleWeight) && (voltageW < threshW)) // if the bottle is on the sensor (patient is not actively using it) and it is almost empty, send warning
  {
    //      Serial.println("WARNING! PATIENT NEEDS REFILL");
    return true;
  }

  return false;
}
/*END OF WATER SECTION*/

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
void printWEB(String error)
{
  if (client)
  {                               // if you get a client,
    Serial.println("new client"); // print a message out the serial port
    String currentLine = "";      // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      {                         // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        Serial.write(c);        // print it out the serial monitor
        if (c == '\n')
        { // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            //create the buttons
            client.print(error);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else
          { // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                   // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
