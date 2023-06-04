// ------------------ ESP-NOW Includes ------------------
#include <esp_now.h>
#include <WiFi.h>
// ------------------------------------------------------

// ------------------ LoRa Includes ------------------
#include <SPI.h>
#include <LoRa.h>
// ------------------------------------------------------
#include "esp_task_wdt.h"


// ------------------ ESP-NOW Variables ------------------
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
// --------------------------------------------------------

// --------------------- Lora Variables ---------------------
#define ss 5
#define rst 14
#define dio0 2
// --------------------------------------------------------

// --------------------- Sensor Variables ---------------------
//DHT 22
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float humidity, temperatur, fahrenheit;

//PH TANAH
#define analogInPin 33
int sensorValue = 0;
float outputValue = 0.0;

//SENSOR TEGANGAN
const int sensorIn = 26;
int mVperAmp = 185;
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

//SENSOR MQ 7
int pin_mq = 27;
int sensorvalue;
float ppm, vrl, rs ;
long rl = 1000;
long ro = 830;
// --------------------------------------------------------


// function to return a JSON string of the sensor data
String makeDataFormat() {
  //   ">data:,temperatur,humidity,AmpsRMS,VRMS,ph_tanah,ppm"

  String data = ">data:,";
  data += temperatur;
  data += ",";
  data += humidity;
  data += ",";
  data += AmpsRMS;
  data += ",";
  data += VRMS;
  data += ",";
  data += outputValue;
  data += ",";
  data += ppm;

  return data;
}


// --------------------- ESP-NOW Functions ---------------------
void esp_now_setup() {

  WiFi.mode(WIFI_STA); // Set WiFi mode to station mode

  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
  }
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      esp_send();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}

void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel

  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&slave, mac, 6);

  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.print("Data received: ");
  Serial.println((char *)incomingData);
}

void esp_send() {
  // make variables to send
  String data = makeDataFormat();

  const uint8_t *peer_addr = slave.peer_addr;

  Serial.print("Sending: "); Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) data.c_str(), data.length());
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNow not Init");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found");
  } else {
    Serial.println("Not sure what happened");
  }
}
// ------------------ End of ESP Now ------------------ //

// ------------------ Lora Function ------------------
void lora_setup() {
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void lora_send() {
  // make variables to send
  String data = makeDataFormat();

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();

  //   print out data send for debuging
  Serial.print("Sending: "); Serial.println(data);


  delay(100);
}
// ------------------ End of Lora Function ------------------

// ------------------ Sensor Function ------------------
void ph_tanah() {
  sensorValue = analogRead(analogInPin);
  outputValue = (-0.016 * sensorValue) + 17.37;
}

void mq7() {
  sensorvalue = analogRead(pin_mq);
  vrl = sensorvalue * 5.00 / 4096; // mengubah nilai ADC ( 0 - 1023 ) menjadi nilai voltase ( 0 - 5.00 volt )
  rs = ( 5.00 * rl / vrl ) - rl;
  ppm = 100 * pow(rs / ro, -1.53); // ppm = 100 * ((rs/ro)^-1.53);

}

void dht22() {
  humidity = dht.readHumidity();
  temperatur = dht.readTemperature();
  fahrenheit = dht.readTemperature(true);

  if (isnan(humidity) || isnan(temperatur) || isnan(fahrenheit)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}

void arus() {
  Serial.println ("");

  // reset the values of the variables.
  Voltage = 0;
  VRMS = 0;
  AmpsRMS = 0;
  Watt = 0;


  Voltage = getVPP();
  VRMS = (Voltage / 2.0) * 0.707; //root 2 is 0.707
  AmpsRMS = ((Voltage * 1000) / mVperAmp) - 0.3; //0.3 is the error I got for my sensor

  // watt = ampere * volt
  Watt = AmpsRMS * VRMS;

  //  Serial.print(Watt);
  //  Serial.println(" Watts");

}

// ***** function calls ******
float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4096;          // store min value here ESP32 ADC resolution

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) //sample for 1 Sec
  {
    readValue = analogRead(sensorIn);
    // see if you have a new maxValue
    if (readValue > maxValue)
    {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue)
    {
      /*record the minimum sensor value*/
      minValue = readValue;
    }
  }

  // Subtract min from max
  result = ((maxValue - minValue) * 3.3) / 4096.0; //ESP32 ADC resolution 4096

  return result;
}
// ------------------ End of Sensor Function ------------------

// ------------------ Misc Function ------------------

// delay in minutes
void delay_minutes(int minutes) {
  unsigned long start = millis();
  while (millis() - start <= minutes * 60 * 1000UL) {
  }
}

// delay in seconds
void delay_seconds(int seconds) {
  // with millis();
  unsigned long start = millis();
  while (millis() - start <= seconds * 1000UL) {
  }
}

// ------------------ End of Misc Function ------------------




// ------------------ Main Function ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Sender Lora & Wifi ESP32");

  dht.begin();

  lora_setup();
}

void loop() {

  // get data from sensor and send it to receiver using lora
  ph_tanah();
  dht22();
  mq7();
  arus();
  lora_send();


  //   print all variable
  Serial.print("Voltage lora: "); Serial.print(Voltage); Serial.println(" V"); // VPP adalah tegangan puncak ke puncak AC
  Serial.print("VRMS lora: "); Serial.print(VRMS); Serial.println(" V"); // VRMS adalah tegangan efektif AC
  Serial.print("AmpsRMS lora: "); Serial.print(AmpsRMS); Serial.println(" A"); // AmpsRMS adalah arus efektif AC
  Serial.print("Watt lora: "); Serial.print(Watt); Serial.println(" W"); // Watt adalah daya AC

  // give delay for the receiver to process incoming data and change the receiver's mode using esp_now
  delay_seconds(5);

  // get data from sensor and send it to receiver using esp_now
  ph_tanah();
  dht22();
  mq7();
  arus();
  esp_now_setup();

  //   print all variable
  Serial.print("Voltage esp: "); Serial.print(Voltage); Serial.println(" V"); // VPP adalah tegangan puncak ke puncak AC
  Serial.print("VRMS esp: "); Serial.print(VRMS); Serial.println(" V"); // VRMS adalah tegangan efektif AC
  Serial.print("AmpsRMS esp: "); Serial.print(AmpsRMS); Serial.println(" A"); // AmpsRMS adalah arus efektif AC
  Serial.print("Watt esp: "); Serial.print(Watt); Serial.println(" W"); // Watt adalah daya AC

  Serial.println("=====================================");


  //   send data every 30 minutes using millis()
  delay_minutes(1);
  //  delay_seconds(5);

  ESP.restart();

}
// ------------------ End of Main Function ------------------
