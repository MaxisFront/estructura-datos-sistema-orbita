#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// Pines NRF24L01
#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";


// Pines de control
#define JOY_Y_PIN A0 // Joystick Izquierdo (Adelante/Atrás)
#define JOY_X_PIN A1 // Joystick Derecho (Izquierda/Derecha)
#define BTN_RETORNO 2 // Botón para accionar la Pila LIFO


struct PaqueteDatos {
  int ejeY;
  int ejeX;
  bool btnRetorno;
};


PaqueteDatos datos;


void setup() {
  pinMode(BTN_RETORNO, INPUT_PULLUP);
 
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}


void loop() {
  datos.ejeY = map(analogRead(JOY_Y_PIN), 0, 1023, -255, 255);
  datos.ejeX = map(analogRead(JOY_X_PIN), 0, 1023, -255, 255);
 
  // Lectura del botón de retorno (Lógica inversa por el INPUT_PULLUP)
  datos.btnRetorno = !digitalRead(BTN_RETORNO);


  // Zonas muertas (Deadband) para evitar que el ruido active los motores
  if (abs(datos.ejeY) < 20) datos.ejeY = 0;
  if (abs(datos.ejeX) < 20) datos.ejeX = 0;


  radio.write(&datos, sizeof(PaqueteDatos));
  delay(10); // Estabilizar la transmisión
}
