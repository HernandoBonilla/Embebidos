/**
 * @file    SistemaMonitoreoFSM_Async.ino
 * @brief   Sistema de monitoreo ambiental con FSM + AsyncTaskLib en ESP32.
 */

#include <Arduino.h>
#include "DHT.h"
#include "StateMachineLib.h"
#include "AsyncTaskLib.h"

////////////////////////////////////////////////////////////
/// ======== PINES (HW) ========
////////////////////////////////////////////////////////////
#define DHTPIN      4
#define DHTTYPE     DHT11
#define LDR_PIN     34
#define BUTTON_PIN  14

#define LED_GREEN   27
#define LED_BLUE    26
#define LED_RED     25

////////////////////////////////////////////////////////////
/// ======== UMBRALES (HIGH / MEDIUM / LOW) ========
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
/// ======== UMBRALES (HIGH / MEDIUM / LOW) ========
////////////////////////////////////////////////////////////

// Temperatura (°C)
#define TEMP_LOW      0.0f
#define TEMP_MEDIUM   25.0f   // Alerta si T > TEMP_MEDIUM
#define TEMP_HIGH     30.0f   // Crítico si T > TEMP_HIGH

// Humedad (%)
#define HUM_LOW       0.0f
#define HUM_MEDIUM    60.0f   // Alerta si H < HUM_MEDIUM
#define HUM_HIGH      100.0f

// Luz (%)
#define LUZ_LOW       0
#define LUZ_MEDIUM    60      // Alerta si Luz < LUZ_MEDIUM
#define LUZ_HIGH      100
////////////////////////////////////////////////////////////
/// ======== LED TIMINGS (6 tareas: ON/OFF por LED) ========
////////////////////////////////////////////////////////////
static const uint16_t GREEN_ON_MS   = 200;
static const uint16_t GREEN_OFF_MS  = 700;

static const uint16_t BLUE_ON_MS    = 500;
static const uint16_t BLUE_OFF_MS   = 900;

static const uint16_t RED_ON_MS     = 100;
static const uint16_t RED_OFF_MS    = 300;

////////////////////////////////////////////////////////////
/// ======== OBJETOS GLOBALES ========
////////////////////////////////////////////////////////////
DHT dht(DHTPIN, DHTTYPE);
StateMachine machine(6, 20);

////////////////////////////////////////////////////////////
/// ======== FSM: Estados y Eventos ========
////////////////////////////////////////////////////////////
enum State { Inicio, MonTemp, MonHum, MonLuz, Alerta, Alarma };
enum Input { Ninguno, Boton, TempAlta, TempCritica, HumBaja, LuzBaja, Timeout };

volatile Input input = Ninguno;

////////////////////////////////////////////////////////////
/// ======== VARIABLES DEL SISTEMA ========
////////////////////////////////////////////////////////////
static bool  sistemaIniciado = false;
static bool  lastButton      = HIGH;

static float ultimaTemp = 0.0f;
static float ultimaHum  = 0.0f;
static int   ultimaLuz  = 0;

static unsigned long stateTime = 0;
static unsigned long lastPrint = 0;

static int  alertCount = 0;

// Evita que se acumulen alertas muy rápido si una condición persiste
static unsigned long lastAlertStamp = 0;
static const unsigned long ALERT_DEBOUNCE_MS = 900;

// ====== NUEVO: flags para NO inflar alertCount ======
static bool countedTemp = false;
static bool countedHum  = false;
static bool countedLuz  = false;

////////////////////////////////////////////////////////////
/// ======== PROTOTIPOS (void) ========
////////////////////////////////////////////////////////////
static void setupMachine(void);

static void onInicio(void);
static void onMonTemp(void);
static void onMonHum(void);
static void onMonLuz(void);
static void onAlerta(void);
static void onAlarma(void);

static void imprimirTabla(void);

static void evaluarTimeout(State st, unsigned long elapsed);
static void registrarAlertaUnaVez(void);

static void evaluarEventosPorEstado(State st);   // solo evalúa eventos

// LEDs 6 tareas
static void stopAllLedTasks(void);
static void updateLedTasks(State st);

// callbacks LEDs
static void GreenOn_cb(void);
static void GreenOff_cb(void);
static void BlueOn_cb(void);
static void BlueOff_cb(void);
static void RedOn_cb(void);
static void RedOff_cb(void);

// Manejo input para no perder eventos
static void consumeInput(void);

////////////////////////////////////////////////////////////
/// ======== ACCIONES DE ESTADO ========
////////////////////////////////////////////////////////////
static void onInicio(void) {
  sistemaIniciado = false;
  alertCount = 0;
  stateTime = millis();
  lastPrint = millis();

  // NUEVO: reset flags para que al iniciar cuente bien
  countedTemp = false;
  countedHum  = false;
  countedLuz  = false;
}

static void onMonTemp(void) {
  sistemaIniciado = true;
  stateTime = millis();

  // NUEVO: permitir 1 alerta por visita a MonTemp
  countedTemp = false;
}

static void onMonHum(void)  {
  stateTime = millis();

  // NUEVO: permitir 1 alerta por visita a MonHum
  countedHum = false;
}

static void onMonLuz(void)  {
  stateTime = millis();

  // NUEVO: permitir 1 alerta por visita a MonLuz
  countedLuz = false;
}

static void onAlerta(void)  { stateTime = millis(); }
static void onAlarma(void)  { stateTime = millis(); }

////////////////////////////////////////////////////////////
/// ======== CONFIGURACIÓN FSM ========
////////////////////////////////////////////////////////////
static void setupMachine(void) {
  machine.AddTransition(Inicio, MonTemp, [](){ return input == Boton; });

  machine.AddTransition(MonTemp, Alerta, [](){ return input == TempAlta; });

  // ====== CAMBIO: QUITADO MonTemp -> Alarma directo ======
  // machine.AddTransition(MonTemp, Alarma, [](){ return input == TempCritica; });

  machine.AddTransition(MonTemp, MonHum, [](){ return input == Timeout; });

  machine.AddTransition(MonHum, Alerta, [](){ return input == HumBaja; });
  machine.AddTransition(MonHum, MonLuz, [](){ return input == Timeout; });

  machine.AddTransition(MonLuz, Alerta, [](){ return input == LuzBaja; });
  machine.AddTransition(MonLuz, MonTemp, [](){ return input == Timeout; });

  // ALARMA: 3 o más alertas + temperatura crítica
  machine.AddTransition(Alerta, Alarma, [](){
    return (alertCount >= 3) && (ultimaTemp > TEMP_HIGH);
  });

  machine.AddTransition(Alerta, MonTemp, [](){ return input == Timeout; });
  machine.AddTransition(Alarma, Inicio, [](){ return input == Boton; });

  machine.SetOnEntering(Inicio, onInicio);
  machine.SetOnEntering(MonTemp, onMonTemp);
  machine.SetOnEntering(MonHum, onMonHum);
  machine.SetOnEntering(MonLuz, onMonLuz);
  machine.SetOnEntering(Alerta, onAlerta);
  machine.SetOnEntering(Alarma, onAlarma);
}

////////////////////////////////////////////////////////////
/// ======== UTILIDADES ========
////////////////////////////////////////////////////////////
static void imprimirTabla(void) {
  Serial.println("==========================================");
  Serial.printf("Temp: %.1f °C\n", ultimaTemp);
  Serial.printf("Hum : %.1f %%\n", ultimaHum);
  Serial.printf("Luz : %d %%\n", ultimaLuz);
  Serial.printf("Alertas acumuladas: %d\n", alertCount);
  Serial.println("==========================================");
}

static void registrarAlertaUnaVez(void) {
  unsigned long now = millis();
  if (now - lastAlertStamp >= ALERT_DEBOUNCE_MS) {
    alertCount++;
    lastAlertStamp = now;
  }
}

////////////////////////////////////////////////////////////
/// ======== EVALUAR EVENTOS (SIN LEER SENSORES AQUÍ) ========
/// Usa valores ya actualizados por la tarea de lectura
////////////////////////////////////////////////////////////
static void evaluarEventosPorEstado(State st) {

  // En alarma no seguir generando eventos por sensores
  if (st == Alarma) return;

  if (st == MonTemp) {
    // Importante: TempCritica ya NO manda directo a Alarma,
    // solo sirve para cumplir la condición en Alerta->Alarma.
    if (!countedTemp && (ultimaTemp > TEMP_MEDIUM)) {
      registrarAlertaUnaVez();
      countedTemp = true;
      input = TempAlta;  // Va a Alerta
    }
  }

  if (st == MonHum) {
    if (!countedHum && (ultimaHum < HUM_MEDIUM)) {
      registrarAlertaUnaVez();
      countedHum = true;
      input = HumBaja;   // Va a Alerta
    }
  }

  if (st == MonLuz) {
    if (!countedLuz && (ultimaLuz < LUZ_MEDIUM)) {
      registrarAlertaUnaVez();
      countedLuz = true;
      input = LuzBaja;   // Va a Alerta
    }
  }
}

////////////////////////////////////////////////////////////
/// ======== TIMEOUTS ========
////////////////////////////////////////////////////////////
static void evaluarTimeout(State st, unsigned long elapsed) {
  if ((st == MonTemp && elapsed >= 3000) ||
      (st == MonHum  && elapsed >= 4000) ||
      (st == MonLuz  && elapsed >= 2000) ||
      (st == Alerta  && elapsed >= 2000)) {
    input = Timeout;
  }
}

////////////////////////////////////////////////////////////
/// ======== LEDs: 6 TASKS (3 ON + 3 OFF) ========
////////////////////////////////////////////////////////////
AsyncTask taskGreenOn (GREEN_ON_MS,  false, GreenOn_cb);
AsyncTask taskGreenOff(GREEN_OFF_MS, false, GreenOff_cb);

AsyncTask taskBlueOn  (BLUE_ON_MS,   false, BlueOn_cb);
AsyncTask taskBlueOff (BLUE_OFF_MS,  false, BlueOff_cb);

AsyncTask taskRedOn   (RED_ON_MS,    false, RedOn_cb);
AsyncTask taskRedOff  (RED_OFF_MS,   false, RedOff_cb);

static void GreenOn_cb(void) {
  digitalWrite(LED_GREEN, HIGH);
  taskGreenOn.Stop(); taskGreenOff.Stop();
  taskGreenOff.Start();
}

static void GreenOff_cb(void) {
  digitalWrite(LED_GREEN, LOW);
  taskGreenOn.Stop(); taskGreenOff.Stop();
  taskGreenOn.Start();
}

static void BlueOn_cb(void) {
  digitalWrite(LED_BLUE, HIGH);
  taskBlueOn.Stop(); taskBlueOff.Stop();
  taskBlueOff.Start();
}

static void BlueOff_cb(void) {
  digitalWrite(LED_BLUE, LOW);
  taskBlueOn.Stop(); taskBlueOff.Stop();
  taskBlueOn.Start();
}

static void RedOn_cb(void) {
  digitalWrite(LED_RED, HIGH);
  taskRedOn.Stop(); taskRedOff.Stop();
  taskRedOff.Start();
}

static void RedOff_cb(void) {
  digitalWrite(LED_RED, LOW);
  taskRedOn.Stop(); taskRedOff.Stop();
  taskRedOn.Start();
}

static void stopAllLedTasks(void) {
  taskGreenOn.Stop(); taskGreenOff.Stop();
  taskBlueOn.Stop();  taskBlueOff.Stop();
  taskRedOn.Stop();   taskRedOff.Stop();

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  digitalWrite(LED_RED,   LOW);
}

static void updateLedTasks(State st) {
  stopAllLedTasks();

  if (st == Inicio) {
    taskGreenOn.Start();
  } else if (st == Alerta) {
    taskBlueOn.Start();
  } else if (st == Alarma) {
    taskRedOn.Start();
  }
}

////////////////////////////////////////////////////////////
/// ======== INPUT: CONSUMIR EVENTO ========
////////////////////////////////////////////////////////////
static void consumeInput(void) {
  input = Ninguno;
}

////////////////////////////////////////////////////////////
/// ======== TAREA 1: LECTURA DE SENSORES ========
/// CAMBIO: NO LEE hasta que se presione botón (sistemaIniciado=true)
////////////////////////////////////////////////////////////
AsyncTask taskReadSensors(2000, true, []() {

  if (!sistemaIniciado) return; // CAMBIO

  // Leer DHT
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h)) ultimaHum = h;
  if (!isnan(t)) ultimaTemp = t;

  // Leer LDR
  int valor = analogRead(LDR_PIN);
  ultimaLuz = map(valor, 0, 4095, 0, 100); // si invertido: 100,0

});

////////////////////////////////////////////////////////////
/// ======== TAREA 2: FSM + BOTÓN ========
////////////////////////////////////////////////////////////
AsyncTask taskFSM(50, true, []() {

  bool actual = digitalRead(BUTTON_PIN);

  if (lastButton == HIGH && actual == LOW) {
    input = Boton;
  }
  lastButton = actual;

  State prev = (State)machine.GetState();
  machine.Update();
  State now = (State)machine.GetState();

  if (now != prev) {
    updateLedTasks(now);
  }

  consumeInput();
});

////////////////////////////////////////////////////////////
/// ======== TAREA 3: EVALUACIÓN + PRINT + TIMEOUT ========
/// CAMBIO: en Inicio NO imprime, NO evalúa
////////////////////////////////////////////////////////////
AsyncTask taskSensoresFSM(200, true, []() {

  State st = (State)machine.GetState();

  //  CAMBIO: En Inicio no se imprime ni se evalúan eventos
  if (st == Inicio) return;

  // Evaluar eventos según estado
  evaluarEventosPorEstado(st);

  // Timeout
  unsigned long elapsed = millis() - stateTime;
  evaluarTimeout(st, elapsed);

  // Print (solo fuera de Inicio)
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    imprimirTabla();
  }
});

////////////////////////////////////////////////////////////
/// ======== SETUP / LOOP ========
////////////////////////////////////////////////////////////
void setup(void) {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LDR_PIN, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  pinMode(LED_RED,   OUTPUT);

  dht.begin();

  setupMachine();
  machine.SetState(Inicio, false, true);

  updateLedTasks((State)machine.GetState());

  taskReadSensors.Start();
  taskFSM.Start();
  taskSensoresFSM.Start();
}

void loop(void) {
  taskReadSensors.Update();
  taskFSM.Update();
  taskSensoresFSM.Update();

  taskGreenOn.Update();  taskGreenOff.Update();
  taskBlueOn.Update();   taskBlueOff.Update();
  taskRedOn.Update();    taskRedOff.Update();
}