// --------------- Include Libraries ---------------
#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <esp_now.h>

#include <SPI.h>
#include <LoRa.h>

#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
// --------------- End Include Libraries ---------------

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// --------------- Define ---------------
#define WIFI_SSID "MF90_452200" //wifi name
#define WIFI_PASSWORD "22345678Aa" //wifi password

#define API_KEY "AIzaSyCquMq3ciMFSZlStLE7DcrKWTZ_-_5MQv4" //firebase api
#define DATABASE_URL "https://iotlora-78392-default-rtdb.asia-southeast1.firebasedatabase.app" //firebase database url

#define ss 5 //pin LoRa
#define rst 14 //pin LoRa
#define dio0 2 //pin LoRa

#define CHANNEL 1 //esp-now channel
// --------------- End Define ---------------


// --------------- Firebase Variables ---------------
FirebaseData fbdo; //firebase data object
FirebaseAuth auth; //firebase auth object
FirebaseConfig config; //firebase config object
bool signupOK = false; //variable for firebase signup
float nil; //variable for firebase data
// --------------- End Firebase Variables ---------------


// ------------------ LoRa Variables ------------------
String dataIn; //variable for LoRa data

String dt[10]; //variable for LoRa data
boolean parsing = false; //variable for LoRa data
int datake = 0; //variable for LoRa data
// ------------------ End LoRa Variables ------------------

// ------------------ ESP-NOW Variables ------------------
String EspNowData; //variable for esp-now data
String EspNowDataIn; //variable for esp-now data

String EspNowDt[10]; //variable for esp-now data
boolean EspNowParsing = false; //variable for esp-now data
int EspNowDatake = 0; //variable for esp-now data
// ------------------ End ESP-NOW Variables ------------------

// ------------------ Timer Variables ------------------
unsigned long sendDataPrevMillis = 0; //variable for timer
// ------------------ End Timer Variables ------------------

bool isLoraReceived = false;
bool isEspNowReceived = false;
unsigned long dataReceivedTime = 0;
const unsigned long DATA_TIMEOUT = 120000; // Waktu timeout setelah 2 menit


// ------------------ WiFi & Firebase Function ------------------
// connect to wifi for sending data to firebase
void connectToExternalWifi() {
  WiFi.mode(WIFI_STA); //set wifi mode to station (client)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

// callback function for firebase token status
void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Sign Up OK");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}
// ------------------ End WiFi & Firebase Function ------------------

// ------------------ LoRa Function ------------------
void setupLoRa() {
  LoRa.setPins(ss, rst, dio0);

  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void parseLoRaData() {
  //   ">data:,temperatur,humidity,AmpsRMS,VRMS,ph_tanah,ppm"
  //   >data:,25.80,63.00,0.27,0.04,-18.23,4.46

  int j = 0;
  dt[j] = "";

  for (int i = 0; i < dataIn.length(); i++) {
    if (dataIn[i] == ',') {
      j++;
      dt[j] = "";
      parsing = true;
    }
    else {
      dt[j] += dataIn[i];
    }
  }

  // dt[0] = ">data:"
  // dt[1] = "temperatur"
  // dt[2] = "humidity"
  // dt[3] = "AmpsRMS"
  // dt[4] = "VRMS"
  // dt[5] = "ph_tanah"
  // dt[6] = "ppm"


  Serial.print("Data length: "); Serial.println(dataIn.length());

  isLoraReceived = true;
}

void receiveLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      dataIn = LoRa.readString();
      Serial.print("Data In: ");
      Serial.println(dataIn);

      parseLoRaData();
    }
  }


}

void sendLoraToFirebase(String ntpTime, String ntpDate, String ntpEpoch) {
  //   ">data:,temperatur,humidity,AmpsRMS,VRMS,ph_tanah,ppm"
  datake++;

  String path = "LoRa-";
  path += "-(";
  path += ntpEpoch;
  path += ")";

  String pathSuhu = path + "/suhu";
  String pathHumidity = path + "/humidity";
  String pathArus = path + "/arus";
  String pathVolt = path + "/volt";
  String pathPh = path + "/ph";
  String pathPpm = path + "/ppm";
  String pathTime = path + "/time";
  String pathDate = path + "/date";

  // dt[0] = ">data:"
  // dt[1] = "temperatur"
  // dt[2] = "humidity"
  // dt[3] = "AmpsRMS"
  // dt[4] = "VRMS"
  // dt[5] = "ph_tanah"
  // dt[6] = "ppm"

  for (int i = 0; i < 7; i++) {
    Serial.print("Data lora ke ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(dt[i]);
  }

  Firebase.RTDB.setString(&fbdo, pathSuhu.c_str(), dt[1]);
  Firebase.RTDB.setString(&fbdo, pathHumidity.c_str(), dt[2]);
  Firebase.RTDB.setString(&fbdo, pathArus.c_str(), dt[3]);
  Firebase.RTDB.setString(&fbdo, pathVolt.c_str(), dt[4]);
  Firebase.RTDB.setString(&fbdo, pathPh.c_str(), dt[5]);
  Firebase.RTDB.setString(&fbdo, pathPpm.c_str(), dt[6]);
  Firebase.RTDB.setString(&fbdo, pathTime.c_str(), ntpTime);
  Firebase.RTDB.setString(&fbdo, pathDate.c_str(), ntpDate);
  Firebase.RTDB.setString(&fbdo, "last data", path.c_str());

  Serial.println("Data sent to Firebase");
  Serial.println();
  Serial.println();

  isLoraReceived = false; //reset lora received
}
// ------------------ End LoRa Function ------------------


// ------------------ ESP-NOW Function ------------------

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

// setup esp-now
void setupESPNow() {
  //Set device in AP mode (access point) with specified SSID
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);


  // Mengabaikan karakter pertama ">data:"
  const char* numericData = reinterpret_cast<const char*>(data) + 6;
  const String rawData = String((char*)data);

  // Menggunakan strtok untuk memisahkan nilai numerik menggunakan delimiter koma
  char* token = strtok(const_cast<char*>(numericData), ",");

  int j = 1;
  EspNowDt[j] = "";

  while (token != NULL) {
    // Mengkonversi token menjadi float
    float value = atof(token);

    // Cetak nilai numerik
    Serial.print("Value: ");
    Serial.println(value);

    // push data to array
    EspNowDt[j] = String(value);
    j++;

    // Pindah ke token berikutnya
    token = strtok(NULL, ",");
  }

  Serial.print("Raw Data: ");
  Serial.println(rawData);


  Serial.println("");

  //   // Cetak hasil parsing
  for (int i = 0; i <= j; i++) {
    Serial.print("Parsed Value ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(EspNowDt[i]);
  }

  Serial.println("");

  // EspNowDt[0] = ">data:"
  // EspNowDt[1] = "temperatur"
  // EspNowDt[2] = "humidity"
  // EspNowDt[3] = "AmpsRMS"
  // EspNowDt[4] = "VRMS"
  // EspNowDt[5] = "ph_tanah"
  // EspNowDt[6] = "ppm"


  Serial.print("Data length: "); Serial.println(EspNowDataIn.length());


  isEspNowReceived = true;
}


void sendEspNowToFirebase(String ntpTime, String ntpDate, String ntpEpoch) {
  //   ">data:,temperatur,humidity,AmpsRMS,VRMS,ph_tanah,ppm"
  datake++;

  String path = "ESP-NOW-";
  path += "-(";
  path += getNtpEpoch();
  path += ")";

  String pathSuhu = path + "/suhu";
  String pathHumidity = path + "/humidity";
  String pathArus = path + "/arus";
  String pathVolt = path + "/volt";
  String pathPh = path + "/ph";
  String pathPpm = path + "/ppm";
  String pathTime = path + "/time";
  String pathDate = path + "/date";

  // EspNowDt[0] = ">data:"
  // EspNowDt[1] = "temperatur"
  // EspNowDt[2] = "humidity"
  // EspNowDt[3] = "AmpsRMS"
  // EspNowDt[4] = "VRMS"
  // EspNowDt[5] = "ph_tanah"
  // EspNowDt[6] = "ppm"

  for (int i = 0; i < 7; i++) {
    Serial.print("Data esp-now ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(EspNowDt[i]);
  }

  Firebase.RTDB.setString(&fbdo, pathSuhu.c_str(), EspNowDt[1]);
  Firebase.RTDB.setString(&fbdo, pathHumidity.c_str(), EspNowDt[2]);
  Firebase.RTDB.setString(&fbdo, pathArus.c_str(), EspNowDt[3]);
  Firebase.RTDB.setString(&fbdo, pathVolt.c_str(), EspNowDt[4]);
  Firebase.RTDB.setString(&fbdo, pathPh.c_str(), EspNowDt[5]);
  Firebase.RTDB.setString(&fbdo, pathPpm.c_str(), EspNowDt[6]);
  Firebase.RTDB.setString(&fbdo, pathTime.c_str(), ntpTime);
  Firebase.RTDB.setString(&fbdo, pathDate.c_str(), ntpDate);
  Firebase.RTDB.setString(&fbdo, "last data", path.c_str());

  isEspNowReceived = false; // reset flag

  setupESPNow();
}

// ------------------ Main Function ------------------
void setup() {
  Serial.begin(115200);
  Serial.println("LoRa & ESP-NOW Receiver then Send to Firebase");

  // lora and esp-now setup stand by mode (to receive data)
  setupESPNow();
  setupLoRa();



}

void loop()  {
  // to string
  Serial.println("isLoraReceived: " + String(isLoraReceived));
  Serial.println("isEspNowReceived: " + String(isEspNowReceived));


  if (isLoraReceived != true) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      while (LoRa.available()) {
        dataIn = LoRa.readString();
        Serial.print("Data In: ");
        Serial.println(dataIn);

        parseLoRaData();
      }
    }
  }

  if (isLoraReceived || isEspNowReceived) {
    unsigned long elapsedTime = millis() - dataReceivedTime;

    // check which data received
    if (isLoraReceived) {
      String printNow = "LoRa data received: ";
      printNow += elapsedTime;
      printNow += " ms";

      Serial.println(printNow);
    } else if (isEspNowReceived) {
      String printNow = "ESP-NOW data received: ";
      printNow += elapsedTime;
      printNow += " ms";
    }


    // 2 minutes
    if (elapsedTime >= DATA_TIMEOUT || (isLoraReceived && isEspNowReceived)) {
      Serial.println("Data timeout, send to firebase");
      connectToExternalWifi();
      setupFirebase();

      Serial.print("waktu sekarang: ");
      Serial.println(getNtpTime());
      Serial.println("tanggal sekarang: ");
      Serial.println(getNtpDate());

      Serial.println("");

		String time = getNtpTime();
		String date = getNtpDate();
		String epoch = getNtpEpoch();

      sendLoraToFirebase(time, date, epoch);
	  sendEspNowToFirebase(time, date, epoch);

      isLoraReceived = false; // reset flag
      isEspNowReceived = false; // reset flag

      // reset timer
      dataReceivedTime = millis();

    }
  }
}
// ------------------ End of Main Function ------------------



// ------------------ Misc Function ------------------
// delay in seconds using millis()
void delaySeconds(unsigned long seconds) {
  unsigned long endTime = millis() + (seconds * 1000);
  while (millis() < endTime) {
    yield();
  }
}

// delay in minutes using millis()
void delayMinutes(unsigned long minutes) {
  unsigned long endTime = millis() + (minutes * 60 * 1000);
  while (millis() < endTime) {
    yield();
  }
}

String getNtpTime() {
  timeClient.begin();
  timeClient.setTimeOffset(25200);  // Sesuaikan dengan offset waktu di zona Anda (dalam detik)
  timeClient.update();

  // Mendapatkan waktu yang terkini
  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();

  return formattedTime;
}

String getNtpEpoch() {
  timeClient.begin();
  timeClient.setTimeOffset(25200);  // Sesuaikan dengan offset waktu di zona Anda (dalam detik)
  timeClient.update();

  // Mendapatkan waktu yang terkini
  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = String(epochTime);

  return formattedTime;
}

String getNtpDate() {
  timeClient.begin();
  timeClient.setTimeOffset(25200);  // Sesuaikan dengan offset waktu di zona Anda (dalam detik)
  timeClient.update();

  // Mendapatkan waktu yang terkini
  unsigned long epochTime = timeClient.getEpochTime();
  tmElements_t timeElements;
  breakTime(epochTime, timeElements);

  // Mendapatkan tanggal dari timeElements
  int year = timeElements.Year + 1970;
  int month = timeElements.Month;
  int day = timeElements.Day;

  Serial.print("Tanggal: ");
  Serial.print(day);
  Serial.print("-");
  Serial.print(month);
  Serial.print("-");
  Serial.println(year);

  String formattedDate = String(day) + "/" + String(month) + "/" + String(year);

  return formattedDate;
}


// ------------------ End of Misc Function ------------------
