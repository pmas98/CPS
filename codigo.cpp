#include <DHT11.h>
#include <MQUnifiedsensor.h>
DHT11 dht11(4);
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 13
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6
int rele=18;

double CO2 = 0;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

void setup() {
  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM = a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862);

  // Configure the ecuation values to get CO2 concentration
  MQ135.init();

  // Start the serial port
  Serial.begin(9600);

  // Calibrate the sensor
  float calcR0 = 0;
  for (int i = 1; i <= 5; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 5);
  Serial.println(" done!");

  // Check for calibration errors
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue found, R0 is infinite (Open circuit detected). Please check your wiring and supply.");
    while (1) {}
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin with short circuit to ground). Please check your wiring and supply.");
    while (1) {}
  }
  pinMode(rele, OUTPUT);
}

void loop() {
  digitalWrite(rele, HIGH);

  int temperature = dht11.readTemperature();
  digitalWrite(12, HIGH);
  int humidity = dht11.readHumidity();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C\tHumidity: ");
  Serial.print(humidity);
  // Update the sensor and read the CO2 level
  MQ135.update();
  CO2 = MQ135.readSensor();

  // Print the CO2 level to the serial monitor
  Serial.print("\tCO2: ");
  Serial.println(CO2);

  // Wait for 5 seconds before printing the next measurement
  delay(500);
  digitalWrite(rele, LOW);
  delay(500);

}
