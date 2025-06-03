#include <WiFi.h>
#include <FirebaseESP32.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- WiFi & Firebase config ---
#define FIREBASE_HOST "https://pfa-project-f33f5-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "XsBkNS2y7w8rBRbxxtOBbmDVyB3zkJYGPhHGb812"
const char* ssid     = "Galaxy A21sA1D5";
const char* password = "12345678";

// --- Brochage personnalisé (identique à ton code test) ---
#define GPS_KEY_PIN   4   // PWRKEY du module SIM808 → D4
#define GPS_RX_PIN    16  // RX2 (vers TX du SIM808)
#define GPS_TX_PIN    17  // TX2 (vers RX du SIM808)
#define I2C_SDA_PIN   23  // SDA du MPU6050
#define I2C_SCL_PIN   22  // SCL du MPU6050

// Firebase
FirebaseData   firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth   firebaseAuth;

// Capteurs
TinyGPSPlus      gps;
HardwareSerial   sim808(2);     // même instance que dans ton test
Adafruit_MPU6050 mpu;

// Variables pas
int   stepCount       = 0;
float previousY       = 0;
bool  stepDetected    = false;
const float stepThreshold = 1.2;

// Stockage des dernières valeurs envoyées pour éviter les envois redondants
float lastLat = 0.0, lastLng = 0.0;
int   lastSteps = 0;

// Envoi vers Firebase (uniquement si changement)
void sendToFirebase(float lat, float lng, int steps) {
  bool sent = false;
  if (fabs(lat - lastLat) > 1e-6) {
    Firebase.setFloat(firebaseData, "/gps/latitude", lat);
    lastLat = lat; sent = true;
  }
  if (fabs(lng - lastLng) > 1e-6) {
    Firebase.setFloat(firebaseData, "/gps/longitude", lng);
    lastLng = lng; sent = true;
  }
  if (steps != lastSteps) {
    Firebase.setInt(firebaseData, "/athlete/steps", steps);
    lastSteps = steps; sent = true;
  }
  if (sent) {
    Serial.printf("Données envoyées → lat: %.6f, lng: %.6f, steps: %d\n", lat, lng, steps);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // 1) Connexion Wi-Fi
  Serial.print("Connexion WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connecté !");

  // 2) Config Firebase
  firebaseConfig.database_url               = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(5);

  // 3) Initialisation I2C (MPU6050)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("Erreur: MPU6050 non détecté !");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.println("MPU6050 initialisé");

  // 4) Initialisation SIM808 pour GPS (identique à ton code test)
  sim808.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("SIM808 (GPS) initialisé sur UART2");

  pinMode(GPS_KEY_PIN, OUTPUT);
  digitalWrite(GPS_KEY_PIN, LOW);
  delay(1000);
  digitalWrite(GPS_KEY_PIN, HIGH);  // démarrage du module
  delay(5000);

  Serial.println("SIM808 prêt. Démarrage du GPS...");
  sim808.println("AT+CGNSPWR=1");   // Allumer GPS
  delay(1000);
  sim808.println("AT+CGNSTST=1");   // Sortie continue des trames NMEA
  delay(1000);
}

void loop() {
  // --- Lecture GPS via TinyGPS++ ---
  while (sim808.available()) {
    gps.encode(sim808.read());
    if (gps.location.isUpdated()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      sendToFirebase(lat, lng, stepCount);
    }
  }

  // --- Détection de pas via MPU6050 ---
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  float y = accel.acceleration.y;
  
  if (!stepDetected && fabs(y - previousY) > stepThreshold) {
    stepCount++;
    sendToFirebase(gps.location.lat(), gps.location.lng(), stepCount);
    stepDetected = true;
  }
  if (fabs(y - previousY) < 0.2) {
    stepDetected = false;
  }
  previousY = y;

  delay(300);  // ajustable
}
