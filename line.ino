//----------ICEBERG ROBOTS----------
//Autor: Finn Harms 2017

#define DEBUG_MODE true  //wenn true -> es werden Statusmeldungen in der Konsole angezeigt

//Über diese 4 digitalen Pins werden die Sensoren am analogen Multiplexer angesteuert, welche ausgelesen werden sollen (indem eine Binärnummer in die vier Pins geschrieben wird)
#define D3 4        //Digitaler Multiplexerpin NR.0
#define D2 10        //Digitaler Multiplexerpin NR.1
#define D1 11        //Digitaler Multiplexerpin NR.2
#define D0 12        //Digitaler Multiplexerpin NR.3

#define SIG1 A6     //Pin, an dem der OUTPUT vom analogen Multiplexer liegt

#define LED_1 8
#define LED_2 7

#define BUTTON 6  //Pin, an dem der Button1 auf der Platine liegt
#define SWITCH 5
#define BUZZER 9

#define INTERRUPT_PIN 2

#include <EEPROM.h>  //über diese Bibliothek können Werte im EEPROM des Arduinos gespeichert werden

byte dPins[] = {D0, D1, D2, D3};    //In diesem Array werden die Digitalen Multiplexerpins gespeichert
int messwerte[16];                  //Array für die Messwerte der einzelnen Sensoren
int feldWerte[16];                  //Für jeden Sensor wird in diesem Array die Helligkeit gespeichert, die jeder einzelne Sensor auf dem grünen Feld misst.
int linienWerte[16];                //Für jeden Sensor wird in diesem Array die Helligkeit gespeichert, die jeder einzelne Sensor auf einem weißen Feld (auf der Linie) misst.
int schwellWerte[16];               //Für jeden Sensor wird in diesem Array die Helligkeit gespeichert, ab der der Sensor als ausgeschlagen gilt. (Mittelwert zwischen feldwerten und linienwerten)
bool aufLinie[16];                  //hier wird gespeichert, welche Sensoren ausgeschlagen. (true -> ausgeschlagen)
bool defekteSensoren[16];           //hier wird gespeichert, welche Sensoren als defekt gelten

bool sensorAktiv = true;            //dieses Boolean gibt an, ob die Sensoren ausgelesen werden sollen
bool linie = false;                 //gibt an, ob eine Linie erkannt wurde
int positionLinie = -1;             //hier wird die Position der Linie gespeichert (die Nummer des Sensors, in wessen Richtung die Linie liegt)
byte sektor[] = {0, 0, 0, 0};       //Anzahl der ausgeschlagenen Sensoren pro Sektor {rechts, vorne, links, hinten}

/*  Speicherbelegung EEPROM
    ByteNr   ->  Variabel
    0 -> schwellWerte[0]/4
    1 -> schwellWerte[1]/4
    2 -> schwellWerte[2]/4
    3 -> schwellWerte[3]/4
   .. -> ...
   12 -> schwellWerte[12]/4
   13 -> schwellWerte[13]/4
   14 -> schwellWerte[14]/4
   15 -> schwellWerte[15]/4
   16
   17
   18


*/

//-- in der Setup-Methode werden die PinModes gesetzt und der Sensor kalibriert
void setup() {
  Serial.begin(115200);                     //startet die Serielle Kommunikation
  debugPrintln("--ICEBERG ROBOTS--");       //schreibt "--ICEBERG ROBOTS--" in die Serielle Konsole
  debugPrintln("Programm gestartet.");      //schreibt Statusmeldung in die Serielle Konsole

  pinMode(INTERRUPT_PIN, OUTPUT);                       //setzt den Interruptpin für den Mega als OUTPUT

  pinMode(D0, OUTPUT);                      //macht den 1. der 4 Digitalen Multiplexerpins zum OUTPUT
  pinMode(D1, OUTPUT);                      //macht den 2. der 4 Digitalen Multiplexerpins zum OUTPUT
  pinMode(D2, OUTPUT);                      //macht den 3. der 4 Digitalen Multiplexerpins zum OUTPUT
  pinMode(D3, OUTPUT);                      //macht den 4. der 4 Digitalen Multiplexerpins zum OUTPUT

  pinMode(SIG1, INPUT);              //macht den Pin, an dem das analoge Signal vom 1. Multiplexer anliegt zum INPUT; der PullUp-Wiederstand wird hinzugeschaltet, da der Fotowiederstand vom analogen Pin zum GND hängt

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SWITCH, INPUT_PULLUP);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  debugPrintln("PinModes gesetzt.");        //schreibt Statusmeldung in die Serielle Konsole
  delay(100);                               //kurze Wartezeit

  if (!digitalRead(BUTTON)) {             //Wenn der Button 1 gedrückt wird, können die Werte neu kalibriert werden
    while (!digitalRead(BUTTON)) {}       //es wird gewartet, bis der Button losgelassen wird
    kalibrieren();                          //ruft die Methode auf, die die Sensoren kalibriert
  } else {
    Serial.println("WERTE AUS EEPROM");     //Gibt die aktuellen kalibrierten Werte in der Konsole aus
    Serial.println("================");
    for (int i = 0; i < 16; i++) {
      schwellWerte[i] = EEPROM.read(i) * 4;   //Liest den Wert aus dem EEPROM an Stelle (Byte) nr. i
      Serial.print("Sensor Nr. ");
      Serial.print(i);
      Serial.print(" -> Schwellw. ");
      Serial.print(schwellWerte[i] * 4);
      defekteSensoren[i] = (schwellWerte[i] == 0); //Es wird ausgelesen, ob der Sensor defekt ist
      if (defekteSensoren[i]) {
        Serial.print(" DEFEKT!");
      }
      Serial.println();
    }
  }
}

//-- in der Loop-Methode wird durchgehend geprüft, ob sich der Roboter auf einer Linie befindet. In diesem Fall wird dem Arduino Mega ein Interrupt gesendet, gefolgt von der Richtung der Linie
void loop() {
  sensorAktiv = digitalRead(SWITCH);
  if (sensorAktiv) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    messen();
    if (linie) {
      senden();
      interrupt();
      delay(20);
    }
  } else {
    ledBlink(LED_BUILTIN, 500);
    ledBlink(LED_1, 210);
    ledBlink(LED_2, 200);
  }
}

//sendet die Werte an den Arduino Mega
void senden() {
  Serial.write(positionErmitteln());
}

//-- Diese Methode gibt die Messwerte zu debugging-Zwecken in der Konsole aus
void ausgeben() {

  //alle Sensoren werden durchgegangen und ausgelesen
  for (int i = 0; i < 16; i++) {
    debugPrint(String(messwerte[i]) + ",");
  }
  debugPrintln("");
}

//-- Diese Methode nimmt Messwerte auf
void messen() {
  int state;        //ein Integer in dem der Zusatng eines Pins gespeichert wird (HIGH/LOW)
  linie = false;

  sektor[0] = 0;
  sektor[1] = 0;
  sektor[2] = 0;
  sektor[3] = 0;

  for (int counter = 0; counter < 16; counter++) { //Der counter zaehlt die Eingabezahl für den analogen Multiplexer
    String binNumber = String(counter, BIN);     //counter wird in Binaer umgewandelt
    byte binLength = binNumber.length();

    //die Binaernummer wird am Anfang mit nullen ergaenzt
    for (int i = 0; i < 4 - binLength; i++) {
      binNumber = "0" + binNumber;
    }

    //die Binaerzahl wird in Steuersignalle fuer die Pins umgewandelt
    for (int i = 0; i < 4; i++) {
      if (binNumber[i] == '0') state = LOW;
      if (binNumber[i] == '1') state = HIGH;
      digitalWrite(dPins[i], state);
    }

    delayMicroseconds(20);


    messwerte[counter] = analogRead(SIG1);

    if (messwerte[counter] < schwellWerte[counter]) {
      aufLinie[counter] = true;
      linie = true;
      if (counter <= 2 || counter >= 14) {
        sektor[1]++;
      }
      if (counter <= 6 && counter >= 2) {
        sektor[2]++;
      }
      if (counter <= 10 && counter >= 6) {
        sektor[3]++;
      }
      if (counter <= 14 && counter >= 10) {
        sektor[0]++;
      }
    } else {
      aufLinie[counter] = false;
    }

  }
}

//-- Diese Methode ermöglicht es, die Sensoren zu Kalibrieren
void kalibrieren() {
  Serial.println("Warte auf Button1 zum Kalibrieren der Feldhelligkeit.");
  while (digitalRead(BUTTON)) {
    ledBlink(LED_BUILTIN, 100);
  }
  messen();
  for (int i = 0; i < 16; i++) {
    feldWerte[i] = messwerte[i];
    defekteSensoren[i] = false;
    aufLinie[i] = false;
  }
  Serial.println("Feldwerte kalibriert.");
  Serial.println("");
  delay(1000);

  Serial.println("Warte auf Button1 zum Kalibrieren der Linienhelligkeit.");

  while (digitalRead(BUTTON)) {
    ledBlink(LED_BUILTIN, 500);
  }

  messen();
  for (int i = 0; i < 16; i++) {
    linienWerte[i] = messwerte[i];
  }
  Serial.println("Linienwerte Kalibriert.");
  Serial.println("");
  delay(1000);

  Serial.println("Schwellwerte werden ausgerechnet...");
  for (int i = 0; i < 16; i++) {
    schwellWerte[i] = (linienWerte[i] + 2 * feldWerte[i]) / 3; //Schwellwerte werden berechnet
    if (schwellWerte[i] / 4 > 255) {
      EEPROM.write(i, 255);  //Werte werden m EEPROM gespeichert
    } else {
      EEPROM.write(i, schwellWerte[i] / 4); //Werte werden m EEPROM gespeichert
    }
    delayMicroseconds(5);
  }
  Serial.println("Schwellwerte berechnet.");

  Serial.println("");
  Serial.println("");
  Serial.println("Sensor:x (Feld/Linie)");

  for (int i = 0; i < 16; i++) {
    Serial.print("Sensor: ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(feldWerte[i]);
    Serial.print("/");
    Serial.print(linienWerte[i]);
    Serial.print(") --> Schwellwert: ");
    Serial.print(schwellWerte[i]);

    if (linienWerte[i] >= feldWerte[i] - 40) {
      Serial.print(" DEFEKT!!! ");
      defekteSensoren[i] = true;
      EEPROM.write(i, 0);
      delayMicroseconds(5);
    }

    Serial.println("");
  }
}

//-- Diese Methode sendet einen Interrupt an den Arduino Mega
void interrupt() {
  digitalWrite(INTERRUPT_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(INTERRUPT_PIN, LOW);
  delayMicroseconds(20);
}

//-- Diese Methode ermittelt die Richtung in der die Linie liegt
byte positionErmitteln() {
  byte lineSektor = 255;
  byte maxAnzahl = 0;
  byte output = 0;
  byte gegenueber = 0;

  for (int i = 0; i < 16; i++) {
    if (aufLinie[i]) {
      output += i;
      maxAnzahl++;
    }
  }
  output /= max(1, maxAnzahl);
  gegenueber = output + 8;
  if (gegenueber >= 16) {
    gegenueber -= 16;
  }

  if (standardabweichung(output) < standardabweichung(gegenueber)) {
    return output;
  } else {
    return gegenueber;
  }

  /*for (int i = 0; i < 4; i++) {
    if (sektor[i] > maxAnzahl) {
      maxAnzahl = sektor[i];
      lineSektor = i;
    }
    }
    return lineSektor;*/
}

byte standardabweichung(byte wert) {
  byte abweichung = 0;
  byte maxAnzahl = 0;
  byte output = 0;

  for (int i = 0; i < 16; i++) {
    if (aufLinie[i]) {
      if (abs(wert - i) > 8) {
        output += 16 - abs(wert - i);
      } else {
        output += abs(wert - i);
      }
      maxAnzahl++;
    }
  }
  output /= max(1, maxAnzahl);
  return output;
}

//-- Lässt eine LED in einer bestimmten Frequenz blinken
void ledBlink(int pin, int freq) {
  digitalWrite(pin, millis() % (2 * freq) > freq);
}

//-- Diese Methode führt ein Serial.print des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrint(String text) {
  if (DEBUG_MODE) {
    Serial.print(text);
  }
}

//-- Diese Methode führt ein Serial.print des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrint(int text) {
  if (DEBUG_MODE) {
    Serial.print(text);
  }
}

//-- Diese Methode führt ein Serial.println des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrintln(String text) {
  if (DEBUG_MODE) {
    Serial.println(text);
  }
}

//-- Diese Methode führt ein Serial.println des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrintln(int text) {
  if (DEBUG_MODE) {
    Serial.println(text);
  }
}

