
/*
========================================================
 PROYECTO 6 - FASE 4
 CONTROL DE BRAZO ROBOTICO
 ESP32 + REMOTEXY + PCA9685
 DOS SERVOMOTORES MG90S

 FUNCIONES:
 - Joystick fisico y virtual
 - Movimiento suavizado
 - Zona muerta de 1800 a 2150
 - Control por coordenadas
 - HOME
 - Secuencias automaticas
 - Sensores de corriente y posicion
 - Paro de emergencia
 - LED y buzzer de alarma
 - Servos en reposo hasta recibir una orden
========================================================
*/

//======================================================
// 1. LIBRERIAS
//======================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <RemoteXY.h>
#include <Adafruit_PWMServoDriver.h>

//======================================================
// 2. CONFIGURACION REMOTEXY
//======================================================

#pragma pack(push, 1)

uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,11,0,12,0,131,0,19,0,0,0,0,31,1,106,200,1,1,11,0,
  5,4,9,60,60,32,2,26,31,4,6,78,14,36,0,2,26,4,27,78,
  14,36,0,2,26,7,54,86,40,10,118,64,2,26,2,7,54,103,40,10,
  118,64,2,26,2,1,73,44,24,24,0,12,31,72,79,77,69,0,1,73,
  14,24,24,0,1,31,69,45,83,84,79,80,0,67,7,132,40,10,78,2,
  26,1,67,7,146,40,10,78,2,26,1,67,7,160,40,10,78,2,26,3,
  1,63,139,24,24,0,2,8,83,69,67,85,69,78,67,73,65,0
};

struct {
  int8_t joystick_01_x;
  int8_t joystick_01_y;

  int8_t slider_servo1;
  int8_t slider_servo2;

  int16_t edit_servo1;
  int16_t edit_servo2;

  uint8_t button_home;
  uint8_t button_stop;
  uint8_t button_start;

  float value_servo1;
  float value_servo2;
  float value_corriente;

  uint8_t connect_flag;

} RemoteXY;

#pragma pack(pop)

//======================================================
// 3. CONFIGURACION PCA9685
//======================================================

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_PWMServoDriver pca =
  Adafruit_PWMServoDriver(0x40);

WebServer server(80);

#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1

#define SERVO_FREQ 50

#define SERVO_MIN_US 600
#define SERVO_MAX_US 2300

#define ANGULO_MIN 10
#define ANGULO_MAX 170

//======================================================
// 4. PINES DE LOS SENSORES Y ALARMA
//======================================================

#define JOY_X 34
#define JOY_Y 35

#define ACS_PIN 33

#define POT1_PIN 32
#define POT2_PIN 36

// LED y buzzer
#define LED_ALARMA 25
#define BUZZER 26

//======================================================
// 5. CONFIGURACION DEL JOYSTICK FISICO
//======================================================

#define ZONA_MIN 1800
#define ZONA_MAX 2150

#define PASO_ANGULO 1

const unsigned long INTERVALO_JOYSTICK = 20;

//======================================================
// 6. CONFIGURACION ACS712
//======================================================

// ACS712 de 5A: 185 mV/A
// ACS712 de 20A: 100 mV/A
// ACS712 de 30A: 66 mV/A

const float ACS_SENSIBILIDAD = 185.0;
const float ACS_DIVISOR = 2.0;

const float LIMITE_CORRIENTE = 2.0;

float ceroACS = 0;
float corriente = 0;

//======================================================
// 7. CALIBRACION DE POTENCIOMETROS
//======================================================

int POT1_MIN = 0;
int POT1_MAX = 4095;

int POT2_MIN = 0;
int POT2_MAX = 4095;

int posicionReal1 = 90;
int posicionReal2 = 90;

//======================================================
// 8. VARIABLES DE LOS SERVOS
//======================================================

int angulo1 = 90;
int angulo2 = 90;

int objetivo1 = 90;
int objetivo2 = 90;

bool servosActivos = false;
bool controlesInicializados = false;

//======================================================
// 9. ESTADOS DEL SISTEMA
//======================================================

bool emergencia = false;
bool alarma = false;

bool secuenciaActiva = false;
bool esperando = false;

bool anteriorHome = false;
bool anteriorStop = false;
bool anteriorStart = false;

//======================================================
// 10. CONTROL DE TIEMPOS
//======================================================

unsigned long tiempoMovimiento = 0;
unsigned long tiempoSecuencia = 0;
unsigned long tiempoJoystick = 0;

const unsigned long INTERVALO_MOVIMIENTO = 15;

//======================================================
// 11. SECUENCIA AUTOMATICA
//======================================================

struct Paso {
  int servo1;
  int servo2;
  unsigned long espera;
};

Paso pasos[5] = {
  {45, 90, 1000},
  {90, 120, 1500},
  {135, 45, 1000}
};

int TOTAL_PASOS = 3;
int pasoActual = 0;

//======================================================
// 12. DETECCION DE CAMBIOS
//======================================================

int ultimoSlider1 = -1;
int ultimoSlider2 = -1;

int ultimoEdit1 = -1;
int ultimoEdit2 = -1;

//======================================================
// 13. FUNCIONES PCA9685
//======================================================

int pulsoMicrosegundos(int angulo) {

  angulo = constrain(
    angulo,
    ANGULO_MIN,
    ANGULO_MAX
  );

  return map(
    angulo,
    0,
    180,
    SERVO_MIN_US,
    SERVO_MAX_US
  );
}

void escribirServo(uint8_t canal, int angulo) {

  int pulso = pulsoMicrosegundos(angulo);

  int ticks = (int)(
    (pulso * 4096.0) / 20000.0
  );

  pca.setPWM(canal, 0, ticks);
}

//======================================================
// 14. ACTIVAR SERVOMOTORES
//======================================================

void activarServos() {

  if (emergencia || servosActivos) {
    return;
  }

  servosActivos = true;

  escribirServo(SERVO1_CHANNEL, angulo1);
  escribirServo(SERVO2_CHANNEL, angulo2);

  tiempoMovimiento = millis();

  Serial.println("SERVOMOTORES ACTIVADOS");
}

//======================================================
// 15. DESACTIVAR SERVOMOTORES
//======================================================

void desactivarServos() {

  servosActivos = false;

  pca.setPWM(SERVO1_CHANNEL, 0, 4096);
  pca.setPWM(SERVO2_CHANNEL, 0, 4096);

  Serial.println("SENALES PWM DESACTIVADAS");
}

//======================================================
// 16. CONTROL DEL LED Y BUZZER
//======================================================

void activarAlarmaVisual() {

  digitalWrite(LED_ALARMA, HIGH);
  digitalWrite(BUZZER, HIGH);

  Serial.println("LED Y BUZZER ACTIVADOS");
}

void desactivarAlarmaVisual() {

  digitalWrite(LED_ALARMA, LOW);
  digitalWrite(BUZZER, LOW);

  Serial.println("LED Y BUZZER DESACTIVADOS");
}

//======================================================
// 17. CALIBRAR ACS712
//======================================================

void calibrarACS() {

  Serial.println("Calibrando ACS712...");

  long suma = 0;

  for (int i = 0; i < 300; i++) {

    suma += analogReadMilliVolts(ACS_PIN);

    delay(3);
  }

  ceroACS = (float)suma / 300.0;

  Serial.print("Voltaje cero: ");
  Serial.println(ceroACS);
}

//======================================================
// 18. MEDIR CORRIENTE
//======================================================

void leerCorriente() {

  static unsigned long ultimaLectura = 0;

  if (millis() - ultimaLectura < 20) {
    return;
  }

  ultimaLectura = millis();

  long suma = 0;

  for (int i = 0; i < 8; i++) {
    suma += analogReadMilliVolts(ACS_PIN);
  }

  float voltajeADC = suma / 8.0;

  float diferencia = abs(
    voltajeADC - ceroACS
  );

  float voltajeSensor =
    diferencia * ACS_DIVISOR;

  float nuevaCorriente =
    voltajeSensor / ACS_SENSIBILIDAD;

  corriente =
    corriente * 0.8 +
    nuevaCorriente * 0.2;
}

//======================================================
// 19. LEER POTENCIOMETROS
//======================================================

void leerPosiciones() {

  int lectura1 = analogRead(POT1_PIN);
  int lectura2 = analogRead(POT2_PIN);

  posicionReal1 = map(
    lectura1,
    POT1_MIN,
    POT1_MAX,
    0,
    180
  );

  posicionReal2 = map(
    lectura2,
    POT2_MIN,
    POT2_MAX,
    0,
    180
  );

  posicionReal1 = constrain(
    posicionReal1, 0, 180
  );

  posicionReal2 = constrain(
    posicionReal2, 0, 180
  );
}

//======================================================
// 20. ACTUALIZAR INDICADORES REMOTEXY
//======================================================

void actualizarIndicadores() {

  RemoteXY.value_servo1 = posicionReal1;
  RemoteXY.value_servo2 = posicionReal2;
  RemoteXY.value_corriente = corriente;
}

//======================================================
// 21. MOVIMIENTO SUAVE
//======================================================

void moverServos() {

  if (emergencia) {
    return;
  }

  if (!servosActivos) {
    return;
  }

  if (millis() - tiempoMovimiento <
      INTERVALO_MOVIMIENTO) {
    return;
  }

  tiempoMovimiento = millis();

  if (angulo1 < objetivo1) {
    angulo1++;
  }
  else if (angulo1 > objetivo1) {
    angulo1--;
  }

  if (angulo2 < objetivo2) {
    angulo2++;
  }
  else if (angulo2 > objetivo2) {
    angulo2--;
  }

  escribirServo(SERVO1_CHANNEL, angulo1);
  escribirServo(SERVO2_CHANNEL, angulo2);
}

//======================================================
// 22. FUNCION HOME
//======================================================

void irHome() {

  if (emergencia) return;

  secuenciaActiva = false;
  esperando = false;

  activarServos();

  objetivo1 = 90;
  objetivo2 = 90;

  Serial.println("REGRESANDO A HOME");
}

//======================================================
// 23. PARO DE EMERGENCIA
//======================================================

void activarEmergencia() {

  if (emergencia) return;

  emergencia = true;
  alarma = true;

  secuenciaActiva = false;
  esperando = false;

  objetivo1 = angulo1;
  objetivo2 = angulo2;

  // Activar LED y buzzer
  activarAlarmaVisual();

  // Desactivar senales PWM
  desactivarServos();

  Serial.println("EMERGENCIA ACTIVADA");
}

//======================================================
// 24. RESTABLECER SISTEMA
//======================================================

void restablecerSistema() {

  if (corriente >= LIMITE_CORRIENTE * 0.8) {

    Serial.println(
      "No se puede restablecer: corriente elevada"
    );

    return;
  }

  emergencia = false;
  alarma = false;

  // Apagar LED y buzzer
  desactivarAlarmaVisual();

  // No activar automaticamente los servos
  servosActivos = false;

  desactivarServos();

  Serial.println("SISTEMA RESTABLECIDO");
  Serial.println("Esperando una nueva orden");
}

//======================================================
// 25. INICIAR SECUENCIA
//======================================================

void iniciarSecuencia() {

  if (emergencia) {
    return;
  }

  activarServos();

  pasoActual = 0;
  esperando = false;

  secuenciaActiva = true;

  objetivo1 = pasos[0].servo1;
  objetivo2 = pasos[0].servo2;

  Serial.println("SECUENCIA INICIADA");
}

//======================================================
// 26. EJECUTAR SECUENCIA SIN DELAY
//======================================================

void ejecutarSecuencia() {

  if (!secuenciaActiva || emergencia) {
    return;
  }

  if (!esperando &&
      angulo1 == objetivo1 &&
      angulo2 == objetivo2) {

    esperando = true;
    tiempoSecuencia = millis();
  }

  if (esperando &&
      millis() - tiempoSecuencia >=
      pasos[pasoActual].espera) {

    esperando = false;
    pasoActual++;

    if (pasoActual >= TOTAL_PASOS) {

      secuenciaActiva = false;

      Serial.println("SECUENCIA FINALIZADA");
      return;
    }

    objetivo1 = pasos[pasoActual].servo1;
    objetivo2 = pasos[pasoActual].servo2;

    Serial.print("PASO: ");
    Serial.println(pasoActual + 1);
  }
}

//======================================================
// 27. CONTROL DEL JOYSTICK FISICO
//======================================================

void controlarJoystickFisico() {

  if (emergencia || secuenciaActiva) {
    return;
  }

  // Controlar frecuencia de lectura
  if (millis() - tiempoJoystick <
      INTERVALO_JOYSTICK) {
    return;
  }

  tiempoJoystick = millis();

  int x = analogRead(JOY_X);
  int y = analogRead(JOY_Y);

  // EJE X: SERVO 1, CANAL 0

  if (x < ZONA_MIN) {

    activarServos();

    objetivo1 -= PASO_ANGULO;
  }
  else if (x > ZONA_MAX) {

    activarServos();

    objetivo1 += PASO_ANGULO;
  }

  // EJE Y: SERVO 2, CANAL 1

  if (y < ZONA_MIN) {

    activarServos();

    objetivo2 -= PASO_ANGULO;
  }
  else if (y > ZONA_MAX) {

    activarServos();

    objetivo2 += PASO_ANGULO;
  }

  // Limitar angulos
  objetivo1 = constrain(
    objetivo1,
    ANGULO_MIN,
    ANGULO_MAX
  );

  objetivo2 = constrain(
    objetivo2,
    ANGULO_MIN,
    ANGULO_MAX
  );
}

//======================================================
// 28. CONTROL REMOTEXY
//======================================================

void controlarRemoteXY() {

  if (emergencia || secuenciaActiva) {
    return;
  }

  // Inicializar valores al conectar

  if (RemoteXY.connect_flag &&
      !controlesInicializados) {

    ultimoSlider1 = RemoteXY.slider_servo1;
    ultimoSlider2 = RemoteXY.slider_servo2;

    ultimoEdit1 = RemoteXY.edit_servo1;
    ultimoEdit2 = RemoteXY.edit_servo2;

    controlesInicializados = true;

    Serial.println("REMOTEXY CONECTADO");
    Serial.println("Esperando orden de movimiento");

    return;
  }

  if (!RemoteXY.connect_flag) {

    controlesInicializados = false;

    return;
  }

  // SLIDER 1

  if (RemoteXY.slider_servo1 != ultimoSlider1) {

    activarServos();

    objetivo1 = map(
      RemoteXY.slider_servo1,
      0,
      100,
      ANGULO_MIN,
      ANGULO_MAX
    );

    ultimoSlider1 = RemoteXY.slider_servo1;
  }

  // SLIDER 2

  if (RemoteXY.slider_servo2 != ultimoSlider2) {

    activarServos();

    objetivo2 = map(
      RemoteXY.slider_servo2,
      0,
      100,
      ANGULO_MIN,
      ANGULO_MAX
    );

    ultimoSlider2 = RemoteXY.slider_servo2;
  }

  // COORDENADA SERVO 1

  if (RemoteXY.edit_servo1 != ultimoEdit1) {

    activarServos();

    objetivo1 = constrain(
      RemoteXY.edit_servo1,
      ANGULO_MIN,
      ANGULO_MAX
    );

    ultimoEdit1 = RemoteXY.edit_servo1;
  }

  // COORDENADA SERVO 2

  if (RemoteXY.edit_servo2 != ultimoEdit2) {

    activarServos();

    objetivo2 = constrain(
      RemoteXY.edit_servo2,
      ANGULO_MIN,
      ANGULO_MAX
    );

    ultimoEdit2 = RemoteXY.edit_servo2;
  }

  // JOYSTICK VIRTUAL

  int x = RemoteXY.joystick_01_x;
  int y = RemoteXY.joystick_01_y;

  if (abs(x) > 8) {

    activarServos();

    objetivo1 = map(
      x,
      -100,
      100,
      ANGULO_MIN,
      ANGULO_MAX
    );

    objetivo1 = constrain(
      objetivo1,
      ANGULO_MIN,
      ANGULO_MAX
    );
  }

  if (abs(y) > 8) {

    activarServos();

    objetivo2 = map(
      y,
      -100,
      100,
      ANGULO_MAX,
      ANGULO_MIN
    );

    objetivo2 = constrain(
      objetivo2,
      ANGULO_MIN,
      ANGULO_MAX
    );
  }
}

//======================================================
// 29. BOTONES REMOTEXY
//======================================================

void controlarBotones() {

  bool home = RemoteXY.button_home;
  bool stop = RemoteXY.button_stop;
  bool start = RemoteXY.button_start;

  // E-STOP

  if (stop && !anteriorStop) {
    activarEmergencia();
  }

  // HOME

  if (home && !anteriorHome) {

    if (emergencia) {
      restablecerSistema();
    }
    else {
      irHome();
    }
  }

  // SECUENCIA

  if (start && !anteriorStart) {
    iniciarSecuencia();
  }

  anteriorHome = home;
  anteriorStop = stop;
  anteriorStart = start;
}

//======================================================
// 30. SEGURIDAD POR SOBRECORRIENTE
//======================================================

void verificarSeguridad() {

  if (corriente >= LIMITE_CORRIENTE &&
      !emergencia) {

    activarEmergencia();

    Serial.println("ALARMA: SOBRECORRIENTE");
  }
}

//======================================================
// 31. ENDPOINT /MOVER
//======================================================

// Ejemplo:
// /mover?servo1=45&servo2=120

void handleMoverHTTP() {

  if (emergencia) {

    server.send(
      403,
      "text/plain",
      "Sistema bloqueado por emergencia"
    );

    return;
  }

  if (!server.hasArg("servo1") &&
      !server.hasArg("servo2")) {

    server.send(
      400,
      "text/plain",
      "Debe enviar servo1 y/o servo2"
    );

    return;
  }

  activarServos();

  if (server.hasArg("servo1")) {

    objetivo1 = constrain(
      server.arg("servo1").toInt(),
      ANGULO_MIN,
      ANGULO_MAX
    );
  }

  if (server.hasArg("servo2")) {

    objetivo2 = constrain(
      server.arg("servo2").toInt(),
      ANGULO_MIN,
      ANGULO_MAX
    );
  }

  secuenciaActiva = false;
  esperando = false;

  server.send(
    200,
    "text/plain",
    "Orden de movimiento recibida"
  );
}

//======================================================
// 32. ENDPOINT /HOME
//======================================================

void handleHomeHTTP() {

  if (emergencia) {

    server.send(
      403,
      "text/plain",
      "Sistema bloqueado: use /reset"
    );

    return;
  }

  irHome();

  server.send(
    200,
    "text/plain",
    "Regresando a HOME"
  );
}

//======================================================
// 33. ENDPOINT /SECUENCIA
//======================================================

void handleSecuenciaHTTP() {

  if (emergencia) {

    server.send(
      403,
      "text/plain",
      "Sistema bloqueado por emergencia"
    );

    return;
  }

  if (!server.hasArg("n")) {

    server.send(
      400,
      "text/plain",
      "Falta el parametro n"
    );

    return;
  }

  int n = server.arg("n").toInt();

  if (n < 1 || n > 5) {

    server.send(
      400,
      "text/plain",
      "La secuencia admite de 1 a 5 pasos"
    );

    return;
  }

  // Verificar parametros

  for (int i = 0; i < n; i++) {

    String sufijo = "_" + String(i + 1);

    if (!server.hasArg("p1" + sufijo) ||
        !server.hasArg("p2" + sufijo) ||
        !server.hasArg("t" + sufijo)) {

      server.send(
        400,
        "text/plain",
        "Faltan parametros de un paso"
      );

      return;
    }
  }

  // Guardar posiciones

  for (int i = 0; i < n; i++) {

    String sufijo = "_" + String(i + 1);

    pasos[i].servo1 = constrain(
      server.arg("p1" + sufijo).toInt(),
      ANGULO_MIN,
      ANGULO_MAX
    );

    pasos[i].servo2 = constrain(
      server.arg("p2" + sufijo).toInt(),
      ANGULO_MIN,
      ANGULO_MAX
    );

    pasos[i].espera = constrain(
      server.arg("t" + sufijo).toInt(),
      0,
      60000
    );
  }

  TOTAL_PASOS = n;

  iniciarSecuencia();

  server.send(
    200,
    "text/plain",
    "Secuencia iniciada"
  );
}

//======================================================
// 34. ENDPOINT /ESTOP
//======================================================

void handleEstopHTTP() {

  activarEmergencia();

  server.send(
    200,
    "text/plain",
    "PARO DE EMERGENCIA ACTIVADO"
  );
}

//======================================================
// 35. ENDPOINT /RESET
//======================================================

void handleResetHTTP() {

  if (!emergencia) {

    server.send(
      200,
      "text/plain",
      "Sistema ya habilitado"
    );

    return;
  }

  if (corriente >= LIMITE_CORRIENTE * 0.8) {

    server.send(
      403,
      "text/plain",
      "Corriente elevada"
    );

    return;
  }

  restablecerSistema();

  server.send(
    200,
    "text/plain",
    "Sistema restablecido. Esperando orden"
  );
}

//======================================================
// 36. ENDPOINT /DATOS
//======================================================

void handleDatosHTTP() {

  String json = "{";

  json += "\"servo1\":" + String(posicionReal1) + ",";
  json += "\"servo2\":" + String(posicionReal2) + ",";
  json += "\"corriente\":" + String(corriente, 2) + ",";

  json += "\"emergencia\":";
  json += (emergencia ? "true" : "false");

  json += ",\"alarma\":";
  json += (alarma ? "true" : "false");

  json += ",\"servosActivos\":";
  json += (servosActivos ? "true" : "false");

  json += ",\"paso\":";
  json += String(
    secuenciaActiva ? pasoActual + 1 : 0
  );

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

//======================================================
// 37. CONFIGURAR ENDPOINTS HTTP
//======================================================

void configurarEndpointsHTTP() {

  server.on("/", HTTP_GET, []() {

    server.send(
      200,
      "text/plain",
      "PROYECTO 6 FASE 4 - "
      "/mover /home /secuencia "
      "/estop /reset /datos"
    );
  });

  server.on(
    "/mover",
    HTTP_GET,
    handleMoverHTTP
  );

  server.on(
    "/home",
    HTTP_GET,
    handleHomeHTTP
  );

  server.on(
    "/secuencia",
    HTTP_GET,
    handleSecuenciaHTTP
  );

  server.on(
    "/estop",
    HTTP_GET,
    handleEstopHTTP
  );

  server.on(
    "/reset",
    HTTP_GET,
    handleResetHTTP
  );

  server.on(
    "/datos",
    HTTP_GET,
    handleDatosHTTP
  );

  server.begin();

  Serial.println("Servidor HTTP iniciado");
}

//======================================================
// 38. SETUP
//======================================================

void setup() {

  Serial.begin(115200);

  // Configurar LED y buzzer
  pinMode(LED_ALARMA, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Iniciar indicadores apagados
  digitalWrite(LED_ALARMA, LOW);
  digitalWrite(BUZZER, LOW);

  analogReadResolution(12);

  Serial.println();
  Serial.println("INICIANDO PROYECTO 6 FASE 4");

  // PUNTO DE ACCESO REMOTEXY

  RemoteXYNet *net = new CRemoteXYNet_WiFiPoint(
    "RemoteXY",
    "12345678"
  );

  RemoteXYGui *gui = RemoteXYEngine.addGui(
    RemoteXY_CONF_PROGMEM,
    &RemoteXY
  );

  gui->addConnectionServer(net, 6377);

  // INICIALIZAR I2C

  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  // INICIALIZAR PCA9685

  if (!pca.begin()) {

    Serial.println("ERROR: PCA9685 NO DETECTADO");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("PCA9685 DETECTADO");

  pca.setPWMFreq(SERVO_FREQ);

  delay(500);

  // No enviar orden de posicionamiento al iniciar
  desactivarServos();

  servosActivos = false;

  // INICIALIZAR SERVIDOR HTTP

  configurarEndpointsHTTP();

  Serial.println("RemoteXY configurado");
  Serial.println("SSID: RemoteXY");
  Serial.println("Password: 12345678");
  Serial.println("Puerto RemoteXY: 6377");
  Serial.println("Servidor HTTP: 80");
  Serial.println("Direccion: http://192.168.4.1/");

  // CALIBRACION ACS712

  calibrarACS();

  Serial.println("-----------------------------");
  Serial.println("SISTEMA LISTO");
  Serial.println("SERVOS EN REPOSO");
  Serial.println("Joystick: zona muerta 1800-2150");
  Serial.println("LED alarma: GPIO 25");
  Serial.println("Buzzer alarma: GPIO 26");
  Serial.println("ESPERANDO ORDEN DE MOVIMIENTO");
  Serial.println("-----------------------------");
}

//======================================================
// 39. LOOP PRINCIPAL
//======================================================

void loop() {

  // COMUNICACION REMOTEXY

  RemoteXYEngine.handler();

  // SERVIDOR HTTP

  server.handleClient();

  // LEER SENSORES

  leerCorriente();

  leerPosiciones();

  // ACTUALIZAR INDICADORES

  actualizarIndicadores();

  // VERIFICAR SEGURIDAD

  verificarSeguridad();

  // CONTROLAR BOTONES

  controlarBotones();

  // CONTROL MANUAL

  if (!emergencia && !secuenciaActiva) {

    controlarRemoteXY();

    controlarJoystickFisico();
  }

  // EJECUTAR SECUENCIAS

  ejecutarSecuencia();

  // MOVIMIENTO SUAVE

  moverServos();
}
