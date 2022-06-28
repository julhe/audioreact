#include <Arduino.h>

#include <WiFiNINA.h>
#include <PDM.h>
#include <Scheduler.h>

#include <damped_spring.h>

// TODO:
//  -build UI slider/button/float changer function
//  -fix UI on mobile (check other web server example)
//  -expose:
//   -Angular Frequency
//   -Microphone Gain

// TODO-Optional:
// -Check compresson like envelope folower

#define BRIGHTNESS_OUT_PIN A0
#define POTI_INPUT_PIN A1

/*
char ssid[] = "yourNetwork";  // your network SSID (name)
char pass[] = "yourPassword"; // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // your network key Index number (needed only for WEP)
int led = LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
*/

// default number of output channels
static const char channels = 1;
// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;

float brightnessSmoothingAngularFreq = 64.0f;

tDampedSpringMotionParams brightnessSmoothing;

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float InvLerp(float a, float b, float v)
{
  return (v - a) / (b - a);
}

// ================================================================
// Brightness Smoothing Context
// ================================================================

void BrightnesSmoothingCalculateParams()
{
  CalcDampedSpringMotionParams(
      &brightnessSmoothing,
      0.005f,                         // delta time (0.005ms is kinda the duration for one loop)
      brightnessSmoothingAngularFreq, // angular frequency
      1.0f);                          // damping ratio
}

// ================================================================
// Running HTTP Server
// ================================================================

/*
void RunWebServer()
{
  // compare the previous status to the current status
  if (status != WiFi.status())
  {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED)
    {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    }
    else
    {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available(); // listen for incoming clients

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
            client.print("<style>");
            client.print(".container {margin: 0 auto; text-align: center; margin-top: 100px;}");
            client.print("button {color: white; width: 100px; height: 100px;");
            client.print("border-radius: 50%; margin: 20px; border: none; font-size: 20px; outline: none; transition: all 0.2s;}");
            client.print(".red{background-color: rgb(196, 39, 39);}");
            client.print(".green{background-color: rgb(39, 121, 39);}");
            client.print(".blue {background-color: rgb(5, 87, 180);}");
            client.print(".off{background-color: grey;}");
            client.print("button:hover{cursor: pointer; opacity: 0.7;}");
            client.print("*{font-size: 20px; font-family: Verdana; font-weight: bold;}");
            client.print("</style>");

            // the content of the HTTP response follows the header:
            client.print("AngFreq: <button class='red' type='submit' onmousedown='location.href=\"/AF_PLUS\"'>+</button>");
            client.print(brightnessSmoothingAngularFreq);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/AF_MINUS\"'>-</button>");
            client.print("<br>");

            int randomReading = analogRead(A1);
            client.print("Random reading from analog pin: (PENIS) ");
            client.print(randomReading);

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

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /AF_PLUS"))
        {
          brightnessSmoothingAngularFreq += 1.0f;
          BrightnesSmoothingCalculateParams();
        }
        if (currentLine.endsWith("GET /AF_MINUS"))
        {
          brightnessSmoothingAngularFreq -= 1.0f;
          BrightnesSmoothingCalculateParams();
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
*/

// ================================================================
// Processing Audio globals
// ================================================================

int ledFinalBrightness = 0;
float ledBrightness_vel = 0.0f, ledBrightness_pos = 0.0f;
unsigned long lastLoop = 0, deltaTime;

void ProcessAudio()
{
  // Wait for samples to be read
  if (samplesRead)
  {
    unsigned long currentTime = millis();
    deltaTime = currentTime - lastLoop;
    lastLoop = currentTime;

    double average = 0;
    for (int i = 0; i < samplesRead; i++)
    {
      average += (double)sampleBuffer[i];
    }
    average /= (double)samplesRead;
    // Clear the read count
    samplesRead = 0;

    float averageLoudness01 = InvLerp(0.0f, 32767.0f, (float)abs(average));
    Serial.print("averageLoudness01:");
    Serial.print(averageLoudness01);
    Serial.println();

    UpdateDampedSpringMotion(&ledBrightness_pos, &ledBrightness_vel, averageLoudness01, brightnessSmoothing);
    ledFinalBrightness = (int)(ledBrightness_pos * 255.0f);

    Serial.print("ledBrightness out:");
    Serial.print(ledBrightness_pos);
    Serial.println();

    Serial.print("deltaTime:");
    Serial.print(deltaTime);
    Serial.println();

    Serial.print("samplesRead:");
    Serial.print(samplesRead);
    Serial.println();
  }
}
/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata()
{
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

/*
void printWiFiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void SetupWifi()
{
  // ================================================================
  // Web Server
  // ================================================================
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("Please upgrade the firmware");
  }

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING)
  {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

*/

// ================================================================
// Setup
// ================================================================
void setup()
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BRIGHTNESS_OUT_PIN, OUTPUT);

  // while (!Serial);
  //  Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shields
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shields
  if (!PDM.begin(channels, frequency))
  {
    Serial.println("Failed to start PDM!");
    while (1)
      ;
  }

  BrightnesSmoothingCalculateParams();
}

void loop()
{

  ProcessAudio();
  // RunWebServer();

  int modifiedRaw = analogRead(POTI_INPUT_PIN);
  double modifier = mapd(modifiedRaw, 0.0, 1023.0, 0.0, 1.0);
  double brightnessLimited = mapd(ledFinalBrightness, 0.0, 255.0, 0.0, 200.0);

  int out = (int)(brightnessLimited * modifier);

  Serial.print("sound: ");
  Serial.print(brightnessLimited);
  Serial.print(", out: ");
  Serial.print(out);
  Serial.println();

  analogWrite(BRIGHTNESS_OUT_PIN, out);
}