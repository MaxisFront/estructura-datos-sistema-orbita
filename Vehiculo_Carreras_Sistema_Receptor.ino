#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// Configuración NRF24L01
#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";


// Configuración Puente H L298N
#define ENA 5
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENB 6


// --- ESTRUCTURA DE DATOS: PILA (STACK) ---
struct Accion {
  int pwmIzq;
  int pwmDer;
  bool dirIzqAvanza;
  bool dirDerAvanza;
  unsigned long duracionMs;
};


#define MAX_PILA 10
Accion pila[MAX_PILA];
int tope = -1;


// Variables de estado temporal para agrupar movimientos
Accion accionActual = {0, 0, true, true, 0};
unsigned long inicioMovimiento = 0;
bool enMovimiento = false;


struct PaqueteDatos {
  int ejeY;
  int ejeX;
  bool btnRetorno;
};
PaqueteDatos datos;


void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
 
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}


// Función auxiliar para empujar datos a la pila
void pushPila(Accion nuevaAccion) {
  if (nuevaAccion.duracionMs < 150) return; // Ignorar micro-movimientos espasmódicos
 
  if (tope < MAX_PILA - 1) {
    tope++;
    pila[tope] = nuevaAccion;
  } else {
    // Si la pila se llena, desplazamos los elementos (FIFO-like behavior para retener los últimos 10)
    for (int i = 0; i < MAX_PILA - 1; i++) {
      pila[i] = pila[i + 1];
    }
    pila[MAX_PILA - 1] = nuevaAccion;
  }
}


// Control físico del hardware
void controlarMotores(int pwmI, int pwmD, bool avanzaI, bool avanzaD) {
  digitalWrite(IN1, avanzaI ? HIGH : LOW);
  digitalWrite(IN2, avanzaI ? LOW : HIGH);
  analogWrite(ENA, pwmI);
 
  digitalWrite(IN3, avanzaD ? HIGH : LOW);
  digitalWrite(IN4, avanzaD ? LOW : HIGH);
  analogWrite(ENB, pwmD);
}


// Desapilar y ejecutar en reversa
void activarSistemaOrbita() {
  while (tope >= 0) {
    Accion paso = pila[tope];
    tope--; // POP
   
    // Invertir direcciones lógicas para regresar
    controlarMotores(paso.pwmIzq, paso.pwmDer, !paso.dirIzqAvanza, !paso.dirDerAvanza);
    delay(paso.duracionMs);
  }
 
  // Detener el vehículo al finalizar la ruta
  controlarMotores(0, 0, true, true);
  enMovimiento = false;
}


void loop() {
  if (radio.available()) {
    radio.read(&datos, sizeof(PaqueteDatos));
   
    if (datos.btnRetorno) {
      // Si hay un movimiento actual a medias, lo cerramos y guardamos antes de regresar
      if (enMovimiento) {
        accionActual.duracionMs = millis() - inicioMovimiento;
        pushPila(accionActual);
        enMovimiento = false;
      }
      activarSistemaOrbita();
      return; // Salir del loop actual hasta nueva orden
    }


    // --- MEZCLA DIFERENCIAL ---
    // Calcula la mezcla de velocidades. Constrain evita que los valores excedan los límites del PWM.
    int velocidadIzqBruta = datos.ejeY + datos.ejeX;
    int velocidadDerBruta = datos.ejeY - datos.ejeX;
   
    int velIzq = constrain(abs(velocidadIzqBruta), 0, 255);
    int velDer = constrain(abs(velocidadDerBruta), 0, 255);
   
    bool izqAvanza = (velocidadIzqBruta >= 0);
    bool derAvanza = (velocidadDerBruta >= 0);
   
    controlarMotores(velIzq, velDer, izqAvanza, derAvanza);


    // --- LÓGICA DE REGISTRO PARA LA PILA ---
    bool estaDetenido = (velIzq == 0 && velDer == 0);
   
    // Umbral de cambio: Si la potencia varía en más de 40 unidades, consideramos que es un "nuevo" movimiento
    bool cambioSignificativo = (abs(velIzq - accionActual.pwmIzq) > 40) ||
                               (abs(velDer - accionActual.pwmDer) > 40) ||
                               (izqAvanza != accionActual.dirIzqAvanza) ||
                               (derAvanza != accionActual.dirDerAvanza);


    if (!estaDetenido) {
      if (!enMovimiento) {
        // Inicia un nuevo movimiento desde cero
        accionActual = {velIzq, velDer, izqAvanza, derAvanza, 0};
        inicioMovimiento = millis();
        enMovimiento = true;
      }
      else if (cambioSignificativo) {
        // La trayectoria cambió drásticamente. Guardamos el tramo anterior y empezamos a grabar el nuevo.
        accionActual.duracionMs = millis() - inicioMovimiento;
        pushPila(accionActual);
       
        accionActual = {velIzq, velDer, izqAvanza, derAvanza, 0};
        inicioMovimiento = millis();
      }
    }
    else if (enMovimiento) {
      // El vehículo se detuvo. Cerramos el registro.
      accionActual.duracionMs = millis() - inicioMovimiento;
      pushPila(accionActual);
      enMovimiento = false;
      accionActual = {0, 0, true, true, 0};
    }
  }
}
