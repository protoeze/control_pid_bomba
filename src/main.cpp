/*
  Control PID de velocidad - bomba peristáltica
  Arduino Nano (ATmega328P) - PlatformIO

  - El AS5600 (debajo del imán del eje del engranaje grande) MIDE la velocidad.
  - El encoder rotativo SOLO cambia el SETPOINT. El pulsador lo pone en 0.
  - En LAZO CERRADO el PWM del pin 9 lo calcula el PID:
        PWM = Kp*e + Ki*integral(e) + Kd*d/dt(-rpm)  (+ compensación de la trabada)
    con e = setpoint - rpm. Si algo frena el motor, el PID sube el PWM; si se pasa, lo baja.
  - Compensación de la trabada del engranaje: como el AS5600 da la POSICIÓN del engranaje
    grande, el programa aprende en qué parte de la vuelta el PID tiene que empujar de más
    (o de menos) y lo aplica un poco antes de llegar ahí en la vuelta siguiente.
    No cambia las ecuaciones del PID: es un término que se suma aparte (comando F).
    Arranca APAGADA: primero conviene tener el PID estable y después probarla con F.
  - En LAZO ABIERTO (comando A) el PWM lo fijás vos y el PID no actúa.
  - Sin AS5600 (o sin imán) el motor NO arranca: sin medición no hay control.
  - El SETPOINT se configura en CAUDAL (cm3/min) y se convierte a RPM para el PID
    (ver CM3_POR_VUELTA). El PID sigue trabajando en RPM.

  Monitor serie (115200, con fin de línea). Escribí ? para ver la ayuda.
    600      -> setpoint en cm3/min (lazo cerrado)
    P3 / I3 / D0.02 -> cambiar Kp / Ki / Kd  (I0 o D0 apagan esa acción)
    A        -> LAZO ABIERTO con el PWM actual;   A40 -> lazo abierto con PWM fijo de 40 %
                (en lazo abierto el encoder mueve el PWM de a 1 % y el pulsador lo pone en 0)
    C        -> volver a LAZO CERRADO, sin salto en el PWM
    F        -> prende/apaga la compensación de la trabada;   R -> borra lo que aprendió
    V        -> vista legible (cada 0,5 s) <-> gráfico (cada 10 ms: t SP VM RPM PWM separados por TAB)
  Vista legible:
    SP:   150 cm3/min        VM:   148 cm3/min ( 15.0 RPM)        PWM:  52 % ( 48 a  56)        oscila: 14.1 a 16.0 RPM
      SP  = caudal que pediste (encoder o serie)
      VM  = valor medido por el AS5600, como caudal: media móvil simple de los últimos 2 s
      PWM = lo que el PID le manda al motor, de 0 a 100 % (100 % = 12 V): promedio de los
            últimos 0,5 s y, entre paréntesis, el mínimo y el máximo. Si ves (0 a 100) el
            PWM está saltando de punta a punta: las ganancias son demasiado altas.
      oscila = velocidad mínima y máxima de los últimos 0,5 s (sin promediar)
  LCD 1602 I2C (mismo bus: SDA = A4, SCL = A5):
      SP    VM    PWM%
      150   148   52
  Líneas con '#' = mensajes.
*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============ Pines ============
const uint8_t PIN_MOTOR  = 9;    // PWM (Timer1) -> R2 -> gate del IRLZ44N
const uint8_t PIN_ENC_A  = 4;    // encoder rotativo
const uint8_t PIN_ENC_B  = 5;
const uint8_t PIN_ENC_SW = 3;    // pulsador del encoder

// ============ AS5600 ============
const uint8_t AS5600_ADDR = 0x36;
const uint8_t REG_STATUS  = 0x0B;
const uint8_t REG_RAW_ANG = 0x0C;
const uint8_t REG_AGC     = 0x1A;

// ============ Constantes del PID ============
// Con Kp=3 / Ki=3 / Kd=0.02 la bomba oscilaba entre 75 y 300 RPM: con los golpes del
// engranaje (juego entre dientes) P y D llevaban el PWM de 0 a 100 % todo el tiempo.
// Con Kp=0.4 / Ki=0.3 / Kd=0.002 oscilaba entre 97 y 112 RPM a 100 RPM, pero la integral
// era lenta. Estas quedan cerca de esas, con la integral un poco más rápida y sin D
// (la derivada reacciona a cada golpe del engranaje y lo empeora).
// Se pueden retocar sin regrabar desde el monitor: P0.5 / I0.5 / D0
float Kp = 0.5;      // [PWM / RPM]
float Ki = 0.5;      // [PWM / (RPM*s)]
float Kd = 0.0;      // [PWM * s / RPM]

// ============ Tiempos (us, medidos con getTime) ============
const uint32_t T_LECTURA  = 2000;  // leer el AS5600 cada 2 ms (mide bien hasta 15000 RPM)
const uint8_t  N_LECTURAS = 5;     // correr el PID cada 5 lecturas -> Ts = 10 ms

// ============ Otros ============
const int8_t  ENC_DIV  = 4;        // transiciones por clic (si salta de a 2 clics, poné 2)
const float   ALPHA    = 0.5;      // filtro de la medición: y += ALPHA*(x - y)  (1 = sin filtro)
const float   PWM_MAX  = 255;
const uint8_t MAX_FALLOS_I2C = 10; // lecturas fallidas seguidas (20 ms) -> motor apagado
const float   STEP_PWM_LA = 1;     // % de PWM por clic del encoder en lazo abierto

// ============ Caudal (bomba peristáltica) ============
// El PID controla RPM; acá solo se convierte caudal <-> RPM:
//     caudal [cm3/min] = CM3_POR_VUELTA x RPM del eje donde está el imán
// Estimación geométrica: en cada vuelta los rodillos empujan un "chorizo" de líquido
// de sección = interior del tubo y largo = la circunferencia que recorren los rodillos.
const float D_TUBO_CM  = 1.0;      // diámetro INTERIOR del tubo [cm] (si 1 cm es el exterior, medí el interior)
const float R_ROTOR_CM = 2.0;      // radio: centro del eje -> centro del tubo apretado [cm]  <-- MEDILO
const float CM3_POR_VUELTA = (PI * D_TUBO_CM * D_TUBO_CM / 4.0) * (2.0 * PI * R_ROTOR_CM);
// CALIBRACIÓN (la geometría es aproximada): poné un SP, juntá agua 1 minuto en una
// probeta y dividí los cm3 que juntaste por las RPM que muestra el monitor. Ese número
// reemplaza a la fórmula de arriba:   const float CM3_POR_VUELTA = <tu valor>;
// (si el rotor de la bomba está en otro eje que el imán, la calibración lo corrige igual)
const float STEP_CAUDAL = 10;      // cm3/min por clic del encoder
const float MAX_RPM     = 210;     // ~85 % de las ~250 RPM que da al 100 %: deja margen al PID
const float MAX_CAUDAL  = MAX_RPM * CM3_POR_VUELTA;

// ============ Compensación de la trabada del engranaje ============
// La vuelta del engranaje grande se divide en NB_COMP posiciones. En cada una se aprende
// cuánto PWM de más (o de menos) hizo falta, y se aplica COMP_ADELANTO segundos antes.
const uint8_t NB_COMP       = 64;     // posiciones por vuelta (5,6° cada una)
const float   COMP_GANANCIA = 0.3;    // cuánto aprende por pasada (más = más rápido, menos estable)
const float   COMP_ADELANTO = 0.015;  // s: el motor tarda en responder, por eso se adelanta
const float   COMP_MAX      = 50;     // límite de la corrección: ±50 de PWM (±20 %)
bool compActiva = false;              // 'F' la prende/apaga, 'R' borra lo aprendido

// ============ Pantalla ============
const uint32_t T_PANTALLA = 500000; // cada cuánto se muestra una línea en la vista legible (us) = 0,5 s
const uint8_t  N_MEDIA    = 200;    // media móvil simple de VM: 200 muestras x 10 ms = 2 s
#define SEP "        "              // separación entre SP, VM y PWM en el monitor serie (vista legible)
const uint8_t  LCD_ADDR_1 = 0x27;   // dirección típica del LCD I2C (PCF8574)
const uint8_t  LCD_ADDR_2 = 0x3F;   // la otra típica (PCF8574A); se usa la que responda
bool modoGrafico = false;           // 'V' por serie: false = legible, true = números crudos cada 10 ms

// =====================================================================
//  Base de tiempo: Timer2 interrumpe cada 1 ms; getTime() devuelve us
// =====================================================================
volatile uint32_t ticks_ms = 0;

ISR(TIMER2_COMPA_vect) { ticks_ms++; }

uint32_t getTime() {                            // microsegundos, resolución 4 us
  uint8_t sreg = SREG;
  cli();
  uint32_t ms = ticks_ms;
  uint8_t  c  = TCNT2;
  if ((TIFR2 & _BV(OCF2A)) && c < 125) ms++;    // pasó el ms pero la ISR todavía no corrió
  SREG = sreg;
  return ms * 1000UL + c * 4UL;
}

// =====================================================================
//  Encoder rotativo (interrupción por cambio de pin) -> solo SETPOINT
// =====================================================================
volatile int16_t encCount = 0;
volatile uint8_t encPrev  = 0;
float setpointCaudal = 0;          // lo que configurás [cm3/min]
float setpointRPM = 0;             // lo que usa el PID [RPM]
bool  lazoAbierto = false;         // true: el PWM lo fijás vos (comando A) y el PID no actúa
float pctAbierto  = 0;             // PWM en lazo abierto [%]

void fijarCaudal(float q) {        // único lugar donde se cambia el setpoint
  setpointCaudal = constrain(q, 0, MAX_CAUDAL);
  setpointRPM = setpointCaudal / CM3_POR_VUELTA;
}

void fijarPwmAbierto(float pct) { pctAbierto = constrain(pct, 0, 100); }

uint8_t leerEncoder() {
  return (digitalRead(PIN_ENC_A) ? 1 : 0) | (digitalRead(PIN_ENC_B) ? 2 : 0);
}

void encUpdate() {
  static const int8_t tabla[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t cur = leerEncoder();
  encCount += tabla[(encPrev << 2) | cur];
  encPrev = cur;
}

ISR(PCINT0_vect) { encUpdate(); }   // D8..D13
ISR(PCINT1_vect) { encUpdate(); }   // A0..A5
ISR(PCINT2_vect) { encUpdate(); }   // D0..D7  (D4 y D5 caen acá)

void habilitarPCINT(uint8_t pin) {  // sirve para cualquier pin, por si cambiás el cableado
  *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
  PCICR |= _BV(digitalPinToPCICRbit(pin));
}

void atenderEncoder() {
  noInterrupts();
  int16_t clics = encCount / ENC_DIV;
  encCount -= clics * ENC_DIV;
  interrupts();
  if (clics) {
    if (lazoAbierto) fijarPwmAbierto(pctAbierto + clics * STEP_PWM_LA);
    else             fijarCaudal(setpointCaudal + clics * STEP_CAUDAL);
  }
}

void atenderBoton() {               // pulsador: setpoint = 0 (antirrebote 30 ms)
  static bool prev = HIGH;
  static uint32_t tCambio = 0;
  bool sw = digitalRead(PIN_ENC_SW);
  uint32_t ahora = getTime();
  if (sw != prev && ahora - tCambio > 30000UL) {
    tCambio = ahora;
    prev = sw;
    if (sw == LOW) {
      if (lazoAbierto) fijarPwmAbierto(0);
      else             fijarCaudal(0);
    }
  }
}

// =====================================================================
//  AS5600
// =====================================================================
bool leerRegistros(uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;           // repeated start
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dst[i] = Wire.read();         // MSB primero
  return true;
}

bool leerAngulo(uint16_t &ang) {
  uint8_t b[2];
  if (!leerRegistros(REG_RAW_ANG, b, 2)) return false;
  ang = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
  return true;
}

// Verifica el sensor y el imán. Si no responde, lista lo que hay en el bus.
bool detectarAS5600() {
  uint8_t st, agc;
  if (!leerRegistros(REG_STATUS, &st, 1) || !leerRegistros(REG_AGC, &agc, 1)) {
    Serial.print(F("# AS5600 NO responde en 0x36. Dispositivos I2C en el bus:"));
    uint8_t n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.print(F(" 0x")); Serial.print(a, HEX); n++; }
    }
    if (n == 0) Serial.print(F(" ninguno -> revisar SDA=A4, SCL=A5, GND y 5V"));
    Serial.println();
    return false;
  }
  Serial.print(F("# AS5600 OK  iman="));  Serial.print((st >> 5) & 1);
  Serial.print(F(" debil="));             Serial.print((st >> 4) & 1);
  Serial.print(F(" fuerte="));            Serial.print((st >> 3) & 1);
  Serial.print(F(" AGC="));               Serial.print(agc);
  Serial.println(F("  (a 5 V lo ideal es AGC cerca de 128)"));
  if (!(st & 0x20)) {
    Serial.println(F("# No detecta el iman: acercarlo y centrarlo sobre el chip"));
    return false;
  }
  return true;
}

// =====================================================================
//  Medición de velocidad
// =====================================================================
uint16_t angPrev = 0;
int32_t  posCuentas = 0;         // posición acumulada (4096 cuentas por vuelta)
uint32_t tPos = 0;               // getTime() de la última lectura válida
int32_t  posCtrlPrev = 0;        // posición y tiempo en el PID anterior
uint32_t tCtrlPrev = 0;
uint8_t  fallosI2C = 1;          // > 0 obliga a re-referenciar en la próxima lectura
int8_t   sentidoGiro = 1;        // +1 / -1 según hacia dónde crece el ángulo

void muestrearAngulo() {         // se llama cada 2 ms
  uint32_t t = getTime();
  uint16_t a;
  if (!leerAngulo(a)) {
    if (fallosI2C < 255) fallosI2C++;
    return;
  }
  if (fallosI2C) {               // primera lectura o venimos de un fallo: re-referenciar
    fallosI2C   = 0;
    angPrev     = a;
    tPos        = t;
    posCtrlPrev = posCuentas;
    tCtrlPrev   = t;
    return;
  }
  int16_t d = (int16_t)a - (int16_t)angPrev;
  if (d > 2047)       d -= 4096;   // cruce 4095 -> 0
  else if (d < -2048) d += 4096;
  angPrev = a;
  posCuentas += d;
  tPos = t;
}

// Velocidad media desde el PID anterior, con el tiempo REAL entre lecturas.
bool calcularRPM(float &rpm, float &dt) {
  uint32_t dtu = tPos - tCtrlPrev;
  int32_t  dp  = posCuentas - posCtrlPrev;
  posCtrlPrev = posCuentas;
  tCtrlPrev   = tPos;
  if (dtu < T_LECTURA / 2) return false;                      // no hubo lecturas válidas
  if (dp) sentidoGiro = (dp > 0) ? 1 : -1;
  dt  = dtu * 1e-6;
  rpm = fabs((float)dp) * (60.0e6 / 4096.0) / (float)dtu;     // la bomba gira en un solo sentido
  return true;
}

// =====================================================================
//  Compensación de la trabada (aprendida según la posición del engranaje grande)
// =====================================================================
// tablaComp[k] = PWM a sumar cuando el engranaje está en la posición k (en 1/16 de PWM).
// Aprende del propio PID: si en cierta parte de la vuelta el PID tuvo que dar más PWM que
// su promedio (porque ahí se traba), guarda esa diferencia un poco ANTES (COMP_ADELANTO)
// para empujar a tiempo en la vuelta siguiente; y al revés donde el engranaje "se suelta".
// La tabla se usa con media cero: el promedio lo sigue poniendo la integral del PID.
int16_t tablaComp[NB_COMP];
int32_t sumaComp = 0;            // suma de la tabla
float   pidMedio = 0;            // promedio lento (~2 s) de la salida del PID
bool    pidMedioOK = false;

void borrarComp() {
  for (uint8_t k = 0; k < NB_COMP; k++) tablaComp[k] = 0;
  sumaComp = 0;
  pidMedioOK = false;
}

uint8_t posicionComp() { return ((uint32_t)angPrev * NB_COMP) >> 12; }   // 0 .. NB_COMP-1

float compensacion(float rpm) {          // PWM a sumar en este instante
  if (!compActiva || lazoAbierto) return 0;
  float f = (tablaComp[posicionComp()] - (float)sumaComp / NB_COMP) / 16.0;
  if (rpm < 0.5 * setpointRPM && f < 0) f = 0;   // si va muy lento, nunca restar PWM (que no se clave)
  return f;
}

void aprenderComp(float salidaPID, float rpm) {
  if (!compActiva || lazoAbierto || setpointRPM <= 0) return;
  if (!pidMedioOK) { pidMedio = salidaPID; pidMedioOK = true; }
  pidMedio += 0.005 * (salidaPID - pidMedio);
  float g = COMP_GANANCIA * (salidaPID - pidMedio) * 16.0;
  int16_t adelanto = (int16_t)(rpm / 60.0 * COMP_ADELANTO * NB_COMP + 0.5);   // en posiciones
  int16_t j = (int16_t)posicionComp() - sentidoGiro * adelanto;
  const int16_t lim = (int16_t)(COMP_MAX * 16);
  for (int8_t d = -1; d <= 1; d++) {                  // reparte en 3 posiciones (suaviza la tabla)
    uint8_t k = (uint8_t)(((j + d) % NB_COMP + NB_COMP) % NB_COMP);
    float inc = g * (d == 0 ? 0.5 : 0.25);
    int16_t nuevo = constrain(tablaComp[k] + (int16_t)(inc + (inc >= 0 ? 0.5 : -0.5)), -lim, lim);
    sumaComp += nuevo - tablaComp[k];
    tablaComp[k] = nuevo;
  }
}

// =====================================================================
//  Media móvil simple del valor medido (solo para mostrar; el PID no la usa)
// =====================================================================
int16_t  bufMedia[N_MEDIA];       // últimas N_MEDIA velocidades, en décimas de RPM
int32_t  sumaMedia = 0;
uint8_t  iMedia = 0, nMedia = 0;

void reiniciarMedia() { sumaMedia = 0; iMedia = 0; nMedia = 0; }

float mediaMovil(float rpm) {     // agrega una muestra y devuelve el promedio de las últimas N_MEDIA
  int16_t x = (int16_t)constrain(rpm * 10.0 + 0.5, 0, 32767);
  if (nMedia == N_MEDIA) sumaMedia -= bufMedia[iMedia];   // sale la más vieja
  else nMedia++;
  bufMedia[iMedia] = x;
  sumaMedia += x;
  if (++iMedia == N_MEDIA) iMedia = 0;
  return sumaMedia / (10.0 * nMedia);
}

// =====================================================================
//  LCD 1602 I2C  (arriba: "SP    VM    PWM%" o "LA ..." en lazo abierto; abajo los valores)
// =====================================================================
LiquidCrystal_I2C *lcd = nullptr;         // se crea en setup() con la dirección que responda
char   lcdQuiero[2][17] = {"SP    VM    PWM%", "                "};   // lo que se quiere mostrar
char   lcdHay[2][17]    = {"                ", "                "};   // lo que ya está escrito
int8_t lcdFila = -1, lcdCol = -1;          // dónde quedó el cursor del LCD (-1 = no se sabe)

bool responde(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void iniciarLCD() {
  uint8_t addr = responde(LCD_ADDR_1) ? LCD_ADDR_1 : (responde(LCD_ADDR_2) ? LCD_ADDR_2 : 0);
  if (!addr) {
    Serial.println(F("# LCD no responde en 0x27 ni 0x3F (revisar cables). El control anda igual."));
    return;
  }
  lcd = new LiquidCrystal_I2C(addr, 16, 2);
  lcd->init();                             // deja la pantalla en blanco; el texto lo escribe atenderLCD()
  lcd->backlight();
  Serial.print(F("# LCD OK en 0x")); Serial.println(addr, HEX);
}

void lcdTexto(uint8_t fila, const char *s) {   // define el texto de una fila (16 caracteres)
  for (uint8_t i = 0; i < 16; i++) lcdQuiero[fila][i] = *s ? *s++ : ' ';
}

// Escribe de a poco: como mucho 2 caracteres por llamada y solo los que cambiaron.
// Cada carácter por I2C tarda ~1,5 ms; así el LCD no frena las lecturas del AS5600.
void atenderLCD() {
  if (!lcd) return;
  uint8_t escritos = 0;
  for (uint8_t f = 0; f < 2; f++) {
    for (uint8_t i = 0; i < 16 && escritos < 2; i++) {
      if (lcdQuiero[f][i] == lcdHay[f][i]) continue;
      if (lcdFila != (int8_t)f || lcdCol != (int8_t)i) lcd->setCursor(i, f);
      lcd->write((uint8_t)lcdQuiero[f][i]);
      lcdHay[f][i] = lcdQuiero[f][i];
      lcdFila = f;
      lcdCol = i + 1;
      escritos++;
    }
  }
}

// =====================================================================
//  Pantalla (monitor serie + LCD)
// =====================================================================
float    sumPwm = 0, minRpm = 0, maxRpm = 0, minPwm = 0, maxPwm = 0;
uint16_t nPant = 0;
uint32_t tPantalla = 0;

void imprimirNum(float v, int8_t ancho, uint8_t decimales) {   // número con ancho fijo (columnas alineadas)
  char b[12];
  dtostrf(v, ancho, decimales, b);
  Serial.print(b);
}

// Se llama cada 10 ms desde el PID. Cada T_PANTALLA muestra una línea en el monitor
// y actualiza el LCD. PWM se muestra de 0 a 100 % (internamente el PID usa 0..255).
void mostrar(float rpm, float pwm) {
  float rpmMedia = mediaMovil(rpm);
  float pwmPorc  = pwm * 100.0 / PWM_MAX;

  if (modoGrafico) {   // t[s]  SP[cm3/min]  VM[cm3/min, sin promediar]  RPM  PWM[%]  separados por TAB (Excel)
    Serial.print(getTime() * 1e-6, 3);         Serial.print('\t');
    Serial.print(setpointCaudal, 1);           Serial.print('\t');
    Serial.print(rpm * CM3_POR_VUELTA, 1);     Serial.print('\t');
    Serial.print(rpm, 1);                      Serial.print('\t');
    Serial.println(pwmPorc, 1);
    return;
  }
  if (nPant == 0) {                           // arranca un período nuevo de 0,5 s
    minRpm = maxRpm = rpm;
    minPwm = maxPwm = pwmPorc;
    sumPwm = 0;
  }
  sumPwm += pwmPorc;
  nPant++;
  if (rpm < minRpm) minRpm = rpm;
  if (rpm > maxRpm) maxRpm = rpm;
  if (pwmPorc < minPwm) minPwm = pwmPorc;
  if (pwmPorc > maxPwm) maxPwm = pwmPorc;

  uint32_t ahora = getTime();
  if (ahora - tPantalla < T_PANTALLA) return;
  tPantalla = ahora;

  float vm      = rpmMedia * CM3_POR_VUELTA;  // valor medido como caudal (media móvil)
  float pwmProm = sumPwm / nPant;
  nPant = 0;

  if (lazoAbierto) { Serial.print(F("LAZO ABIERTO     " SEP)); }
  else { Serial.print(F("SP:")); imprimirNum(setpointCaudal, 6, 0); Serial.print(F(" cm3/min" SEP)); }
  Serial.print(F("VM:"));   imprimirNum(vm, 6, 0);             Serial.print(F(" cm3/min ("));
  imprimirNum(rpmMedia, 5, 1);                                 Serial.print(F(" RPM)" SEP));
  Serial.print(F("PWM:"));  imprimirNum(pwmProm, 4, 0);        Serial.print(F(" % ("));
  imprimirNum(minPwm, 3, 0); Serial.print(F(" a")); imprimirNum(maxPwm, 4, 0); Serial.print(F(")" SEP));
  Serial.print(F("oscila:")); imprimirNum(minRpm, 6, 1);
  Serial.print(F(" a"));      imprimirNum(maxRpm, 6, 1);       Serial.println(F(" RPM"));

  char fila[17];                              // "SP    VM    PWM%"  ->  "150   148   52  "
  if (lazoAbierto) {
    lcdTexto(0, "LA    VM    PWM%");          // LA = lazo abierto
    snprintf(fila, sizeof(fila), "--    %-5u %-4u", (unsigned)(vm + 0.5), (unsigned)(pwmProm + 0.5));
  } else {
    lcdTexto(0, "SP    VM    PWM%");
    snprintf(fila, sizeof(fila), "%-5u %-5u %-4u",
             (unsigned)(setpointCaudal + 0.5), (unsigned)(vm + 0.5), (unsigned)(pwmProm + 0.5));
  }
  lcdTexto(1, fila);
}

// =====================================================================
//  PID
// =====================================================================
float rpmFilt = 0, rpmFiltPrev = 0;
float termI = 0;                 // término integral, ya en unidades de PWM
float u = 0;                     // salida [PWM 0..255]

void apagarMotor() {
  u = 0;
  termI = 0;
  analogWrite(PIN_MOTOR, 0);
}

void controlar() {
  float rpm, dt;
  if (!calcularRPM(rpm, dt)) return;              // sin datos nuevos: mantener la salida

  rpmFilt += ALPHA * (rpm - rpmFilt);

  if (lazoAbierto) {                                  // LAZO ABIERTO: PWM fijo, el PID no actúa
    u = pctAbierto * PWM_MAX / 100.0;
  } else if (setpointRPM <= 0) {
    u = 0;
    termI = 0;
  } else {
    float ff = compensacion(rpmFilt);                 // compensación de la trabada (0 si está apagada)
    float e = setpointRPM - rpmFilt;                  // error
    float P = Kp * e;
    float D = -Kd * (rpmFilt - rpmFiltPrev) / dt;     // derivada de la medición (sin salto al cambiar el setpoint)
    // Anti-windup: la integral SIEMPRE integra (así corrige la media aunque P sature en un
    // pico) pero queda limitada a 0..255: sola nunca puede pedir más que 0..100 % de PWM.
    // (Antes dejaba de integrar cuando la salida saturaba; con los golpes del engranaje
    //  saturaba casi siempre, la integral quedaba congelada y la media no llegaba al SP.)
    termI = constrain(termI + Ki * e * dt, 0, PWM_MAX);
    float v = P + termI + D + ff;
    u = constrain(v, 0, PWM_MAX);
    if (v >= 0 && v <= PWM_MAX)                       // no aprender mientras la salida satura
      aprenderComp(P + termI + D, rpmFilt);           // aprende de lo que hizo el PID en esta posición
  }
  rpmFiltPrev = rpmFilt;
  analogWrite(PIN_MOTOR, (int)(u + 0.5));

  mostrar(rpmFilt, u);
}

// =====================================================================
//  Monitor serie (no bloquea)
// =====================================================================
// Ganancias en las dos formas: Kp, Ki, Kd (las que usa el código) y Kp, Ti, Td (las de las
// tablas de Ziegler-Nichols):  Ki = Kp / Ti,  Kd = Kp * Td
void imprimirGanancias() {
  Serial.print(F("# Kp=")); Serial.print(Kp, 4);
  Serial.print(F("  Ki=")); Serial.print(Ki, 4);
  Serial.print(F("  Kd=")); Serial.print(Kd, 4);
  Serial.print(F("     (Ti="));
  if (Ki > 0) { Serial.print(Kp / Ki, 3); Serial.print(F(" s")); } else Serial.print(F("sin I"));
  Serial.print(F("  Td="));
  if (Kp > 0 && Kd > 0) { Serial.print(Kd / Kp, 4); Serial.print(F(" s")); } else Serial.print(F("sin D"));
  Serial.println(F(")"));
}

void imprimirModo() {
  if (lazoAbierto) {
    Serial.print(F("# LAZO ABIERTO: PWM fijo en ")); Serial.print(pctAbierto, 0);
    Serial.println(F(" % (el PID no actua). Encoder: +-1 %.  C = volver a lazo cerrado"));
  } else {
    Serial.print(F("# LAZO CERRADO (PID).  Compensacion de la trabada: "));
    Serial.println(compActiva ? F("PRENDIDA (F para apagar)") : F("APAGADA (F para prender)"));
  }
}

void imprimirAyuda() {
  Serial.println(F("# ---------------------------- COMANDOS ----------------------------"));
  Serial.println(F("#  600        setpoint en cm3/min (lazo cerrado)"));
  Serial.println(F("#  P3 I3 D0.02  ganancias Kp, Ki, Kd  (I0 o D0 apagan esa accion)"));
  Serial.println(F("#  A          LAZO ABIERTO con el PWM actual;  A40 = lazo abierto al 40 %"));
  Serial.println(F("#  C          volver a LAZO CERRADO (sin salto)"));
  Serial.println(F("#  F          prende/apaga la compensacion de la trabada;  R = borrar lo aprendido"));
  Serial.println(F("#  V          vista legible <-> grafico (t SP VM RPM PWM, cada 10 ms, separados por TAB)"));
  Serial.println(F("#  ?          esta ayuda"));
  Serial.println(F("# -----------------------------------------------------------------"));
}

void procesarComando(char *s) {
  char c = toupper(s[0]);
  if (c == 'P' || c == 'I' || c == 'D') {
    float v = atof(s + 1);       // la integral está en PWM: cambiar Ki no produce saltos
    if (c == 'P') Kp = v; else if (c == 'I') Ki = v; else Kd = v;
    imprimirGanancias();
  } else if (c == 'A') {                       // LAZO ABIERTO
    if (s[1]) fijarPwmAbierto(atof(s + 1));    // "A40" -> 40 %
    else      fijarPwmAbierto(u * 100.0 / PWM_MAX);   // "A" -> sigue con el PWM que tenía (sin salto)
    lazoAbierto = true;
    imprimirModo();
  } else if (c == 'C') {                       // LAZO CERRADO
    if (lazoAbierto) {
      lazoAbierto = false;
      // Transferencia sin salto: la integral arranca de forma que P + I = PWM que había
      termI = constrain(u - Kp * (setpointRPM - rpmFilt), 0, PWM_MAX);
    }
    imprimirModo();
  } else if (c == 'F') {
    compActiva = !compActiva;
    imprimirModo();
  } else if (c == 'R') {
    borrarComp();
    Serial.println(F("# Compensacion borrada: vuelve a aprender desde cero"));
  } else if (c == 'V') {
    modoGrafico = !modoGrafico;
    nPant = 0;
    tPantalla = getTime();       // la primera línea legible cubre 0,5 s completos
    if (modoGrafico) Serial.println(F("# t[s]\tSP[cm3/min]\tVM[cm3/min]\tRPM\tPWM[%]"));
    else             Serial.println(F("# Vista legible: una linea cada 0,5 s"));
  } else if (isDigit(c) || c == '.') {
    fijarCaudal(atof(s));
  } else {
    imprimirAyuda();
  }
}

void atenderSerial() {
  static char linea[20];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n == 0) continue;
      linea[n] = 0;
      n = 0;
      procesarComando(linea);
    } else if (n < sizeof(linea) - 1) {
      linea[n++] = c;
    }
  }
}

// =====================================================================
bool     sensorOK = false;
uint32_t tProx = 0, tReintento = 0;
uint8_t  nLect = 0;

void setup() {
  pinMode(PIN_MOTOR, OUTPUT);
  analogWrite(PIN_MOTOR, 0);
  TCCR1B = (TCCR1B & 0xF8) | 0x02;   // PWM de D9 a ~3.9 kHz (Timer1; no afecta al resto)

  // Base de tiempo: Timer2 en modo CTC, interrupción cada 1 ms
  TCCR2B = 0;                        // detenerlo mientras se configura
  TCCR2A = _BV(WGM21);               // modo CTC
  OCR2A  = 249;                      // 250 cuentas x 4 us = 1 ms
  TCNT2  = 0;
  TIFR2  = _BV(OCF2A);               // borrar una bandera vieja (si no, cuenta un ms de más)
  TIMSK2 = _BV(OCIE2A);              // interrupción por comparación A
  TCCR2B = _BV(CS22);                // arrancar: prescaler 64 -> 4 us por cuenta

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);             // 100 kHz: lo más robusto con cables y con el LCD en el bus
  Wire.setWireTimeout(3000, true);   // si el bus se cuelga por ruido, la lectura falla en vez de trabar todo
  iniciarLCD();                      // (la biblioteca vuelve a llamar a Wire.begin(): se reconfigura abajo)
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  encPrev = leerEncoder();
  habilitarPCINT(PIN_ENC_A);
  habilitarPCINT(PIN_ENC_B);

  imprimirGanancias();
  Serial.print(F("# Caudal: "));    Serial.print(CM3_POR_VUELTA, 2);
  Serial.print(F(" cm3 por vuelta, maximo ")); Serial.print(MAX_CAUDAL, 0);
  Serial.println(F(" cm3/min"));
  imprimirModo();
  Serial.println(F("# Escribi ? para ver los comandos"));
  borrarComp();
  sensorOK = detectarAS5600();
  tReintento = tProx = getTime();
}

void loop() {
  atenderEncoder();
  atenderBoton();
  atenderSerial();

  uint32_t ahora = getTime();

  // Sin sensor no hay control: motor apagado y se reintenta cada 1 s
  if (!sensorOK) {
    lcdTexto(1, "SIN SENSOR");
    atenderLCD();
    if (ahora - tReintento >= 1000000UL) {
      tReintento = ahora;
      sensorOK = detectarAS5600();
      if (sensorOK) {
        fallosI2C = 1;
        rpmFilt = rpmFiltPrev = 0;
        nLect = 0;
        reiniciarMedia();
        tProx = getTime();
      }
    }
    return;
  }

  if ((int32_t)(ahora - tProx) < 0) return;                   // todavía no pasaron 2 ms
  tProx += T_LECTURA;
  if ((int32_t)(ahora - tProx) >= 0) tProx = ahora + T_LECTURA; // muy atrasados: resincronizar

  muestrearAngulo();
  if (fallosI2C >= MAX_FALLOS_I2C) {
    apagarMotor();
    sensorOK = false;
    Serial.println(F("# Se perdio la lectura del AS5600: motor apagado"));
    return;
  }

  if (++nLect < N_LECTURAS) return;                           // PID cada 10 ms
  nLect = 0;
  controlar();
  atenderLCD();                                               // hasta 2 caracteres del LCD por ciclo
}
