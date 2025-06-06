# Practica-5-PD
## **Practica 5: Buses de comunicación I (introducción y I2c)**

 El objetivo de la practica es comprender el funcionamiento de los buses sistemas de comunicación entre
 periféricos; estos elementos pueden ser internos o externos al procesador.
 Esta es la primera practica de 4 donde se verán los buses i2c, spi, i2s , usart.
 Para lo cual realizaremos una practica en cada caso donde controlaremos un periférico de ejemplo

## **Ejercicio Practico 1**
**Codigo main.cpp:**
```cpp
#include <Arduino.h>
#include <Wire.h>
void setup()
{
  Wire.begin(21,20);
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}
void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(5000);           // wait 5 seconds for next scan
}
```
1. Descibir la salida por el puerto serie
I2C Scanner
Scanning...
I2C device found at address 0x3C  !
done
2. Explicar el funcionamiento
El programa escanea todas las direcciones posibles del bus I2C para detectar dispositivos conectados, enviando una señal a cada dirección y reportando por el puerto serie cuáles responden correctamente. Así permite identificar qué dispositivos I2C están presentes en el sistema.



## **Ejercicio Practico 2**

```cpp
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definir las dimensiones del OLED y la dirección I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No utilizamos un pin de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200); // Inicializar comunicación serie

  // Inicializar la comunicación I2C en los pines SDA (21) y SCL (20) para ESP32
  Wire.begin(21, 20); // Usa los pines 21 (SDA) y 20 (SCL)

  // Inicializar el display OLED con la dirección I2C 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se pudo encontrar un display OLED"));
    for (;;); // Detener el programa si no se encuentra el display
  }

  // Limpiar la pantalla
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Mostrar mensaje inicial
  display.println(F("Escaneando dispositivos I2C..."));
  display.display();
  delay(1000);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  display.clearDisplay(); // Limpiar la pantalla antes de mostrar resultados
  display.setCursor(0, 0); // Ubicar el cursor en la parte superior

  // Escanear direcciones I2C de 1 a 126
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      // Si se encuentra un dispositivo, mostrar su dirección en hexadecimal
      display.print(F("Dispositivo I2C en: 0x"));
      if (address < 16) display.print("0"); // Asegurarse de que la dirección tiene 2 dígitos
      display.println(address, HEX); // Mostrar dirección en formato hexadecimal
      nDevices++;
    } else if (error == 4) {
      // Error desconocido
      display.print(F("Error en: 0x"));
      if (address < 16) display.print("0");
      display.println(address, HEX);
    }
  }

  // Si no se encontraron dispositivos, mostrar mensaje correspondiente
  if (nDevices == 0) {
    display.println(F("No se encontraron dispositivos"));
  } else {
    display.println(F("Escaneo completado"));
  }

  display.display(); // Actualizar pantalla con los resultados
  delay(5000); // Esperar 5 segundos antes de hacer otro escaneo
}
```
Este código escanea el bus I2C en busca de dispositivos conectados y muestra sus direcciones en un display OLED. Usa los pines 21 (SDA) y 20 (SCL) para I2C, y actualiza cada 5 segundos la lista de dispositivos encontrados directamente en la pantalla.



```cpp
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No estamos usando un pin de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {
  Serial.begin(115200);


  // Inicializar la comunicación I2C en los pines 21 (SDA) y 20 (SCL)
  Wire.begin(21, 20);
  
  // Inicializar el display con la dirección I2C 0x3C (o 0x3D dependiendo de tu pantalla)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se pudo encontrar un display OLED"));
    for (;;); // Detener el programa si no se encuentra el display
  }


  // Limpiar la pantalla
  display.clearDisplay();


  // Configurar el texto en la pantalla
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("¡Quiero aprobar!"));
  display.display();
}


void loop() {
  // Lo que quieras actualizar en la pantalla
}

```
Este código inicializa un display OLED conectado vía I2C (pines 21 y 20) y muestra el mensaje “¡Quiero aprobar!” en texto grande en la pantalla. Después de mostrar el mensaje, el programa no realiza actualizaciones adicionales en el loop.

Foto montaje:

![FOTO MONTAJE](https://github.com/user-attachments/assets/a956f85a-4e32-49a5-9260-7570ee6a1b93)



## **Ejercicio PARA SUBIR NOTA**

```cpp
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Definir los pines I2C personalizados para ESP32-S3
#define SDA_PIN 21  
#define SCL_PIN 22

// Configuración OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensor MAX30105
MAX30105 particleSensor;
uint32_t irBuffer[100], redBuffer[100];
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Configurar I2C

  // Iniciar pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Fallo en OLED SSD1306");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();

  // Iniciar sensor MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor MAX30105 no encontrado");
    while (1);
  }

  // Configuración optimizada para MAX30105
  particleSensor.setup(69, 4, 2, 100, 411, 4096);  // Configuración optimizada para HR + SpO2
}

void loop() {
  // Leer 100 muestras del sensor
  for (byte i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calcular SpO2 y HR
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Mostrar en pantalla OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("HR: ");
  display.println(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  display.print("SpO2: ");
  display.println(validSPO2 ? String(spo2) + " %" : "N/A");
  display.display();

  Serial.print("HR: ");
  Serial.print(validHeartRate ? String(heartRate) + " BPM" : "N/A");
  Serial.print(" | SpO2: ");
  Serial.println(validSPO2 ? String(spo2) + " %" : "N/A");

  delay(1000);
}
```
Este código usa un sensor para medir la frecuencia cardíaca (HR) y la saturación de oxígeno en sangre (SpO2). 
Lee 100 muestras del sensor, calcula HR y SpO2 con un algoritmo, y muestra los resultados en un display OLED y en el puerto serie cada segundo.
