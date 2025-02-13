#include <WiFi.h>
#include "DFRobot_MultiGasSensor.h"
#include <Wire.h>
#include "DHTesp.h"
#include <WiFiUdp.h>
#include <ArduinoJson.h>
DHTesp dht;
#define DHTPIN 12     // Digital pin connected to the DHT sensor (replace with your pin number)
#define DHTTYPE DHT22

#if defined(ESP32) || defined(ESP8266)
  // D7 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  #define SENSOR_DATA_PIN   (7)   // Sensor PWM interface
  #define INTERRUPT_NUMBER   digitalPinToInterrupt(SENSOR_DATA_PIN)   // interrupt number
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 5 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  #define SENSOR_DATA_PIN   (5)   // Sensor PWM interface
  #define INTERRUPT_NUMBER   digitalPinToInterrupt(SENSOR_DATA_PIN)   // interrupt number
#else
  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control: 
  // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
  #define SENSOR_DATA_PIN   (2)   // Sensor PWM interface
  #define INTERRUPT_NUMBER   (0)   // interrupt number
#endif

// Used in interrupt, calculate pulse width variable
volatile unsigned long pwmHighStartTicks=0, pwmHighEndTicks=0;
volatile unsigned long pwmHighVal=0, pwmLowVal=0;
// interrupt flag
volatile uint8_t flag=0;

//Enabled by default, use IIC communication at this time. Use UART communication when disabled
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x74
DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
  SoftwareSerial mySerial(0,1);
  DFRobot_GAS_SoftWareUart gas(&mySerial);
#else
  DFRobot_GAS_HardWareUart gas(&Serial1,9600,15,16); //ESP32HardwareSerial
#endif
#endif

const char* ssid = "Steven"; // nombre de la red
const char* password = "steven123"; //clave del wifi
const char* serverIP = "192.168.107.129"; // IP del servidor
const int serverPort = 8000; //puerto de comunicación
void interruptChange()

{
  if (digitalRead(SENSOR_DATA_PIN)) {
    pwmHighStartTicks = micros();    // store the current micros() value
    if(2 == flag){
      flag = 4;
      if(pwmHighStartTicks > pwmHighEndTicks) {
        pwmLowVal = pwmHighStartTicks - pwmHighEndTicks;
      }
    }else{
      flag = 1;
    }
  } else {
    pwmHighEndTicks = micros();    // store the current micros() value
    if(1 == flag){
      flag = 2;
      if(pwmHighEndTicks > pwmHighStartTicks){
        pwmHighVal = pwmHighEndTicks - pwmHighStartTicks;
      }
    }
  }
}
WiFiUDP udp;
void setup() {
  Serial.begin(115200);
  dht.setup(DHTPIN, DHTesp::DHT22);
  // Inicia la conexión a la red WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Conectando a la red WiFi...");
  }

  Serial.println("Conectado a la red WiFi");
  

  Wire.begin(16, 11);

  while(!gas.begin())
  {
    Serial.println("No Devices !");
    delay(1000);
  }
  Serial.println("The device is connected successfully!");

  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);

  gas.setTempCompensation(gas.OFF);

  pinMode(SENSOR_DATA_PIN, INPUT);
  attachInterrupt(INTERRUPT_NUMBER, interruptChange, CHANGE);
  Serial.println("3 min for beginning...");

}




void loop() {

  StaticJsonDocument<200> doc; //texto json donde se guardan los datos recogidos de los sensores
  char buffer[128]; //buffer de datos

  float concentration; //variable que contempla los valores del CO2
  
  
    
  Serial.println("Esperando datos...");
  delay(10000);
  float temp = dht.getTemperature();
  float humidity = dht.getHumidity();
  Serial.print("Temperature: ");Serial.println(temp);
  Serial.print("Humidity: ");Serial.println(humidity);
  Serial.println(flag);
  if(flag == 4){
  flag = 1;
  float pwmHighVal_ms = (pwmHighVal * 1000.0) / (pwmLowVal + pwmHighVal);

  if (pwmHighVal_ms < 0.01){
    Serial.println("Fault");
  }
  else if (pwmHighVal_ms < 80.00){
    Serial.println("preheating");
  }
  else if (pwmHighVal_ms < 998.00){
    concentration = (pwmHighVal_ms - 2) * 5;
    // Print pwmHighVal_ms
    Serial.print("pwmHighVal_ms:");
    Serial.print(pwmHighVal_ms);
    Serial.println("ms");
    //Print CO2 concentration
    Serial.print(concentration);
    Serial.println("ppm");
  }else{
    Serial.println("Beyond the maximum range : 398~4980ppm");
  }
  Serial.println();
}
  Serial.print("Ambient ");
  Serial.print(gas.queryGasType());
  Serial.print(" concentration is: ");
  Serial.print(gas.readGasConcentrationPPM());// línea que muestra el porcentaje de oxígeno en volúmen
  Serial.println(" %vol");

  doc["o2"] =  gas.readGasConcentrationPPM();
  doc["co2"] = concentration ;
  doc["temperature"] = temp;
  doc["humidity"] = humidity;

  
  //Envío de los datos al server

  size_t len = serializeJson(doc, buffer);
  if (concentration > 0){
    udp.beginPacket(serverIP, serverPort);
    udp.write((uint8_t*)buffer, len);
    udp.endPacket();
    Serial.println("Datos enviados: " + String(buffer));
  }
  
      
}

