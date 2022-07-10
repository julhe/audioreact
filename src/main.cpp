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

#define BRIGHTNESS_OUT_PIN A0
#define POTI_INPUT_PIN A1

char ssid[] = "yourNetwork";  // your network SSID (name)
char pass[] = "yourPassword"; // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // your network key Index number (needed only for WEP)
int led = LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

enum PotiMode{
  None,
  LScale,
  LBias,
};

static struct
{
  int mic_hardware_gain = 128;
  double led_max_brightness = 1.0;
  bool smoothed_volume_mode = false;
  PotiMode mode = PotiMode::LScale;
  float smoothModeAngularFreq = 16.0f;
  float smoothModeAngularFreqAttackScale = 2.f;
  float ambSmoothModeAngularFreq = 2.0f;
  float loudnessBias = 1.25f;
  float loudnessScale = 1.0f;
  tDampedSpringMotionParams smoothModeParams;
  tDampedSpringMotionParams smoothModeParamsAttack;
  tDampedSpringMotionParams ambLoudnessParams;
} config;

static struct
{
  int ledFinalBrightness = 0;
  float ledBrightness_vel = 0.0f, ledBrightness_pos = 0.0f;
  float ambLoudness_vel = 0.0f, ambLoudness_pos = 0.0f;
  unsigned long lastLoop = 0, deltaTime;
  double envelopeFollowerGain = 1.0;
} ctx;

// default number of output channels
static const char channels = 1;
// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;

double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
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
      &config.smoothModeParams,
      0.005f,                       // delta time (0.005ms is kinda the duration for one loop)
      config.smoothModeAngularFreq, // angular frequency
      1.0f);                        // damping ratio

  CalcDampedSpringMotionParams(
      &config.smoothModeParamsAttack,
      0.005f,                       // delta time (0.005ms is kinda the duration for one loop)
      config.smoothModeAngularFreq * config.smoothModeAngularFreqAttackScale, // angular frequency
      1.0f);                        // damping ratio
}

void AmbLoudnessSmoothingCalculateParams()
{

  CalcDampedSpringMotionParams(
      &config.ambLoudnessParams,
      0.005f,                          // delta time (0.005ms is kinda the duration for one loop)
      config.ambSmoothModeAngularFreq, // angular frequency
      1.0f);                           // damping ratio
}

// ================================================================
// Running HTTP Server
// ================================================================
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
            client.print(config.smoothModeAngularFreq);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/AF_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("AngFreq AttackScale: <button class='red' type='submit' onmousedown='location.href=\"/AF_AS_PLUS\"'>+</button>");
            client.print(config.smoothModeAngularFreqAttackScale);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/AF_AS_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("Amb AngFreq: <button class='red' type='submit' onmousedown='location.href=\"/AMB_AF_PLUS\"'>+</button>");
            client.print(config.ambSmoothModeAngularFreq);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/AMB_AF_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("Mic Gain: <button class='red' type='submit' onmousedown='location.href=\"/MIC_GAIN_PLUS\"'>+</button>");
            client.print(config.mic_hardware_gain);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/MIC_GAIN_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("Loudness Scale: <button class='red' type='submit' onmousedown='location.href=\"/L_SCALE_PLUS\"'>+</button>");
            client.print(config.loudnessScale);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/L_SCALE_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("Loudness Bias: <button class='red' type='submit' onmousedown='location.href=\"/L_BIAS_PLUS\"'>+</button>");
            client.print(config.loudnessBias);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/L_BIAS_MINUS\"'>-</button>");
            client.print("<br>");

            client.print("Smooth Volume Mode: <button class='blue' type='submit' onmousedown='location.href=\"/SV_MODE\"'>+</button>");
            client.print(config.smoothed_volume_mode);
            client.print("<br>");


            client.print("Pot. Disabled: <button class='blue' type='submit' onmousedown='location.href=\"/P_TO_NONE\"'>+</button>");
            client.print(config.mode == PotiMode::None);
            client.print("<br>");

            client.print("Pot. To Bias: <button class='blue' type='submit' onmousedown='location.href=\"/P_TO_BIAS\"'>+</button>");
            client.print(config.mode == PotiMode::LBias);
            client.print("<br>");

            client.print("Pot. To Scale: <button class='blue' type='submit' onmousedown='location.href=\"/P_TO_SCALE\"'>+</button>");
            client.print(config.mode == PotiMode::LScale);
            client.print("<br>");

            client.print("Max Led Brightness Time: <button class='red' type='submit' onmousedown='location.href=\"/LED_BRIGHT_P\"'>+</button>");
            client.print(config.led_max_brightness);
            client.print(" <button class='green' type='submit' onmousedown='location.href=\"/LED_BRIGHT_M\"'>-</button>");
            client.print("<br>");


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

        // smoothing attack scale
        if (currentLine.endsWith("GET /AF_AS_PLUS"))
        {
          config.smoothModeAngularFreqAttackScale += 0.1f;
          BrightnesSmoothingCalculateParams();
        }
        if (currentLine.endsWith("GET /AF_AS_MINUS"))
        {
          config.smoothModeAngularFreqAttackScale -= 0.1f;
          BrightnesSmoothingCalculateParams();
        }

        // smoothing
        if (currentLine.endsWith("GET /AF_PLUS"))
        {
          config.smoothModeAngularFreq += 1.0f;
          BrightnesSmoothingCalculateParams();
        }
        if (currentLine.endsWith("GET /AF_MINUS"))
        {
          config.smoothModeAngularFreq -= 1.0f;
          BrightnesSmoothingCalculateParams();
        }

        // AMB af smoothing
        if (currentLine.endsWith("GET /AMB_AF_PLUS"))
        {
          config.ambSmoothModeAngularFreq += 0.25f;
          AmbLoudnessSmoothingCalculateParams();
        }
        if (currentLine.endsWith("GET /AMB_AF_MINUS"))
        {
          config.ambSmoothModeAngularFreq -= 0.25f;
          AmbLoudnessSmoothingCalculateParams();
        }

        // loudness scale
        if (currentLine.endsWith("GET /L_SCALE_PLUS"))
        {
          config.loudnessScale += 0.01f;
        }
        if (currentLine.endsWith("GET /L_SCALE_MINUS"))
        {
          config.loudnessScale -= 0.01f;
        }

        // loudness bias
        if (currentLine.endsWith("GET /L_BIAS_PLUS"))
        {
          config.loudnessBias += 0.2f;
        }
        if (currentLine.endsWith("GET /L_BIAS_MINUS"))
        {
          config.loudnessBias -= 0.1f;
        }


        // led
        if (currentLine.endsWith("GET /LED_BRIGHT_P")) {
          config.led_max_brightness += 0.1f;
        }
        if (currentLine.endsWith("GET /LED_BRIGHT_M")) {
          config.led_max_brightness -= 0.1f;
        }


        // mic hardware gain
        if (currentLine.endsWith("GET /MIC_GAIN_PLUS"))  {
          config.mic_hardware_gain += 4;
          PDM.setGain(config.mic_hardware_gain);
        }
        if (currentLine.endsWith("GET /MIC_GAIN_MINUS")) {
          config.mic_hardware_gain -= 1;
          PDM.setGain(config.mic_hardware_gain);
        }

// switch mode
        if (currentLine.endsWith("GET /SV_MODE")){
          config.smoothed_volume_mode = !config.smoothed_volume_mode;
        }


        if (currentLine.endsWith("GET /P_TO_NONE")) {
          config.mode = PotiMode::None;
        }

        if (currentLine.endsWith("GET /P_TO_BIAS")) {
          config.mode = PotiMode::LBias;
        }

        if (currentLine.endsWith("GET /P_TO_SCALE")) {
          config.mode = PotiMode::LScale;
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

// ================================================================
// Processing Audio globals
// ================================================================

void ProcessAudio()
{
  // Wait for samples to be read
  if (samplesRead)
  {
    unsigned long currentTime = millis();
    ctx.deltaTime = currentTime - ctx.lastLoop;
    ctx.lastLoop = currentTime;

    // compute the highest loudness over the past time
    short min_value = 32767, max_value = -32768;
    double average_value = 0.0;
    for (int i = 0; i < samplesRead; i++)
    {
      short s = sampleBuffer[i];
      min_value = min(min_value, s);
      max_value = max(max_value, s);
      average_value += (double)s;
    }

    int loudness_raw = (int)max_value - (int)min_value;
    average_value /= (double)samplesRead;
    // Clear the read count
    samplesRead = 0;

    float loudness01 = InvLerp(0, 32767.0f, (float)loudness_raw);
    float valueMin = InvLerp(-32767.0f, 32767.0f, (float)min_value);
    float valueMax = InvLerp(-32767.0f, 32767.0f, (float)max_value);

    // compute the "ambient" loudness
    UpdateDampedSpringMotion(&ctx.ambLoudness_pos, &ctx.ambLoudness_vel, loudness01, config.ambLoudnessParams);


    int modifiedRaw = analogRead(POTI_INPUT_PIN);
    float modifiedRawFloat = (float) modifiedRaw;
    if(config.mode == PotiMode::LBias){
     
      float biasByPoti = mapf(modifiedRawFloat, 0.0f, 1023.0f, -2.0f, 2.0f);
      config.loudnessBias = biasByPoti;
      Serial.print("loudnessBias:");
    Serial.print(biasByPoti);
    Serial.println();
    }

    if(config.mode  == PotiMode::LScale){
      float scaleByPoti = mapf((float) modifiedRaw, 0.0f, 1023.0f, 0.0f, 4.0f);
      config.loudnessScale = scaleByPoti;
    }


    float normalizedLoudness = loudness01 * (0.5f / ctx.ambLoudness_pos);
    float biasedNormalizedLoudness = max(0.0f, normalizedLoudness * (config.loudnessScale* 2.0f) - (0.5f + config.loudnessBias));

    loudness01 = biasedNormalizedLoudness;

  
    
    // gain = constrain(gain, 0, 255);
    // PDM.setGain(gain);



    // Serial.print("ambient loudness");
    // Serial.print(ctx.ambLoudness_pos * 32767.0f);
    // Serial.println();

    // Serial.print("normalizedLoudness:");
    // Serial.print(normalizedLoudness * 32767.0f);
    // Serial.println();

    // Serial.print("min value:");
    // Serial.print(min_value);
    // Serial.println();

    // Serial.print("max value:");
    // Serial.print(max_value);
    // Serial.println();

    // Serial.print("samplesRead:");
    // Serial.print(samplesRead);
    // Serial.println();

    // config.smoothModeAngularFreq = mapd(modifiedRaw, 0.0, 1023.0, 1.0, 96.0);
    // BrightnesSmoothingCalculateParams();

    // double brightnessLimited = mapd(ledFinalBrightness, 0.0, 255.0, 0.0, 200.0);

    if (config.smoothed_volume_mode)
    {
      UpdateDampedSpringMotion(&ctx.ledBrightness_pos, &ctx.ledBrightness_vel, loudness01, config.smoothModeParams);
      ctx.ledFinalBrightness = (int)(ctx.ledBrightness_pos * 255.0f);
    }
    else
    {
      const double thresshold = 0.5;
      double overshoot = abs(thresshold - loudness01);

      if (loudness01 > thresshold)
      {
        UpdateDampedSpringMotion(&ctx.ledBrightness_pos, &ctx.ledBrightness_vel, loudness01, config.smoothModeParamsAttack);
      }
      else
      {
        UpdateDampedSpringMotion(&ctx.ledBrightness_pos, &ctx.ledBrightness_vel, loudness01, config.smoothModeParams);
                                                    // Serial.println("bellow");
      }

      ctx.envelopeFollowerGain = max(0.0, min(1.0, ctx.envelopeFollowerGain));
      ctx.ledFinalBrightness = (int)(ctx.ledBrightness_pos * 255.0f);
    }

    // Serial.print("envFollowGain:");
    // Serial.print(envelopeFollowerGain);
    // Serial.println();
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
  PDM.setGain(128);
  BrightnesSmoothingCalculateParams();
  AmbLoudnessSmoothingCalculateParams();

  SetupWifi();
}

void loop()
{

  ProcessAudio();
  RunWebServer();

  double ledFinalBrightness = min((config.led_max_brightness * (double)ctx.ledFinalBrightness), 200);
  //ledFinalBrightness = sqrt((ledFinalBrightness) / 255.0) * 255.0;
  analogWrite(BRIGHTNESS_OUT_PIN, ledFinalBrightness);
}
