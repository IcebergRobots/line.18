//----------ICEBERG ROBOTS----------
//Autor: Finn Harms 2017

#define DEBUG_MODE true  //wenn true -> es werden Statusmeldungen in der Konsole angezeigt

//Über diese 4 digitalen Pins werden die Sensoren am analogen Multiplexer angesteuert, welche ausgelesen werden sollen (indem eine Binärnummer in die vier Pins geschrieben wird)
#define D0 4        //Digitaler Multiplexerpin NR.0
#define D1 10        //Digitaler Multiplexerpin NR.1
#define D2 11        //Digitaler Multiplexerpin NR.2
#define D3 12        //Digitaler Multiplexerpin NR.3

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
byte sektor[] = {0,0,0,0};

/*  Speicherbelegung EEPROM
 *  ByteNr   ->  Variabel
 *  0 -> schwellWerte[0]
 *  1 -> schwellWerte[1]
 *  2 -> schwellWerte[2]
 *  3 -> schwellWerte[3]
 * .. -> ...
 * 28 -> schwellWerte[28]
 * 29 -> schwellWerte[29]
 * 30 -> schwellWerte[30]
 * 31 -> schwellWerte[31]
 * 32 
 * 33
 * 34
 * 
 * 
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
  kalibrieren();
}

//-- in der Loop-Methode wird durchgehend geprüft, ob sich der Roboter auf einer Linie befindet. In diesem Fall wird dem Arduino Mega ein Interrupt gesendet, gefolgt von der Richtung der Linie
void loop() {
  sensorAktiv = digitalRead(SWITCH);
  if(sensorAktiv){
    digitalWrite(LED_BUILTIN, HIGH);
    messen();
    digitalWrite(BUZZER, linie);
    if(linie){
      interrupt();
      senden();
    }
  }else{
    ledBlink(LED_BUILTIN, 200);
  }
}

//sendet die Werte an den Arduino Mega
void senden(){
  
}

//-- Diese Methode gibt die Messwerte zu debugging-Zwecken in der Konsole aus
void ausgeben(){

  //alle Sensoren werden durchgegangen und ausgelesen
  for(int i = 0; i<16; i++){
    debugPrint(String(messwerte[i])+",");
  }
  debugPrintln("");
}

//-- Diese Methode nimmt Messwerte auf
void messen(){
  int state;        //ein Integer in dem der Zusatng eines Pins gespeichert wird (HIGH/LOW)
  linie = false;
  
  for(int counter = 0; counter < 16; counter++){ //Der counter zaehlt die Eingabezahl für den analogen Multiplexer 
    String binNumber = String(counter, BIN);     //counter wird in Binaer umgewandelt
    byte binLength = binNumber.length();         

    //die Binaernummer wird am Anfang mit nullen ergaenzt
    for(int i = 0; i< 4-binLength; i++){          
      binNumber = "0" + binNumber;                
    }

    //die Binaerzahl wird in Steuersignalle fuer die Pins umgewandelt
    for(int i = 0; i<4; i++){
      if(binNumber[i] == '0') state = LOW;
      if(binNumber[i] == '1') state = HIGH;
      digitalWrite(dPins[i], state);
    }

    delayMicroseconds(20);
    
    messwerte[counter] = analogRead(SIG1);
    
    if(messwerte[counter] < schwellWerte[counter]){
      aufLinie[counter] = true;
      linie = true;
    }else{
      aufLinie[counter] = false;
    }
   
  }
}

//-- Diese Methode ermöglicht es, die Sensoren zu Kalibrieren
void kalibrieren(){
  Serial.println("Warte auf Button1 zum Kalibrieren der Feldhelligkeit.");
  while(digitalRead(BUTTON)){
    ledBlink(LED_1, 100);
  }
  messen();
  for(int i = 0; i < 32; i++){
    feldWerte[i] = messwerte[i];
    defekteSensoren[i] = false;
    aufLinie[i] = false;
  }
  Serial.println("Feldwerte kalibriert.");
  Serial.println("");
  delay(1000);

  Serial.println("Warte auf Button1 zum Kalibrieren der Linienhelligkeit.");
  
  while(digitalRead(BUTTON)){
    ledBlink(LED_1, 500);
  }
  
  messen();
  for(int i = 0; i < 32; i++){
    linienWerte[i] = messwerte[i];
  }
  Serial.println("Linienwerte Kalibriert.");
  Serial.println("");
  delay(1000);

  Serial.println("Schwellwerte werden ausgerechnet...");
  for(int i = 0; i< 32; i++){
    schwellWerte[i] = (2*linienWerte[i]+feldWerte[i])/3;  //Schwellwerte werden berechnet
    if(schwellWerte[i]>255){
      EEPROM.write(i, 255);  //Werte werden m EEPROM gespeichert
    }else{
      EEPROM.write(i, schwellWerte[i]);  //Werte werden m EEPROM gespeichert
    }
    delayMicroseconds(5);
  }
  Serial.println("Schwellwerte berechnet.");

  Serial.println("");
  Serial.println("");
  Serial.println("Sensor:x (Feld/Linie)");

  for (int i = 0; i< 32; i++){
    Serial.print("Sensor: ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(feldWerte[i]);
    Serial.print("/");
    Serial.print(linienWerte[i]);
    Serial.print(") --> Schwellwert: ");
    Serial.print(schwellWerte[i]);

    if(linienWerte[i] >= feldWerte[i]-40){
      Serial.print(" DEFEKT!!! ");
      defekteSensoren[i] = true;
      EEPROM.write(i, 0);
      delayMicroseconds(5);
    }
    
    Serial.println("");
  }  
}

//-- Diese Methode sendet einen Interrupt an den Arduino Mega
void interrupt(){   
  digitalWrite(INTERRUPT_PIN,HIGH);
  delayMicroseconds(20);
  digitalWrite(INTERRUPT_PIN,LOW);
  delayMicroseconds(20);
}

//-- Diese Methode ermittelt die Richtung in der die Linie liegt
byte positionErmitteln(){
  int pos = 255;
  for(int i = 0; i<16; i++){
    
  }
  return pos;
}

//-- Lässt eine LED in einer bestimmten Frequenz blinken
void ledBlink(int pin, int freq){
  digitalWrite(pin, millis() % 2*freq > freq);
}

//-- Diese Methode führt ein Serial.print des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrint(String text){
  if(DEBUG_MODE){
    Serial.print(text);
  }
}

//-- Diese Methode führt ein Serial.print des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrint(int text){
  if(DEBUG_MODE){
    Serial.print(text);
  }
}

//-- Diese Methode führt ein Serial.println des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrintln(String text){
  if(DEBUG_MODE){
    Serial.println(text);
  }
}

//-- Diese Methode führt ein Serial.println des Textes aus, wenn das Macro DEBUG_MODE auf true gesetzt ist.
void debugPrintln(int text){
  if(DEBUG_MODE){
    Serial.println(text);
  }
}
