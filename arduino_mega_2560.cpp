 /***************************************************************************************************
 *	 █████╗ ██████╗ ██████╗ ██╗   ██╗██╗███╗   ██╗ ██████╗     ██████╗ ███████╗ ██████╗  ██████╗ 
 *	██╔══██╗██╔══██╗██╔══██╗██║   ██║██║████╗  ██║██╔═══██╗    ╚════██╗██╔════╝██╔════╝ ██╔═████╗
 *	███████║██████╔╝██║  ██║██║   ██║██║██╔██╗ ██║██║   ██║     █████╔╝███████╗███████╗ ██║██╔██║
 *	██╔══██║██╔══██╗██║  ██║██║   ██║██║██║╚██╗██║██║   ██║    ██╔═══╝ ╚════██║██╔═══██╗████╔╝██║
 *	██║  ██║██║  ██║██████╔╝╚██████╔╝██║██║ ╚████║╚██████╔╝    ███████╗███████║╚██████╔╝╚██████╔╝
 *	╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝  ╚═════╝ ╚═╝╚═╝  ╚═══╝ ╚═════╝     ╚══════╝╚══════╝ ╚═════╝  ╚═════╝  
 *
 *                     			REMOTE CONTROLLED AIRSOFT TURRET 
 *								  - ARDUINO MEGA CONTROLLER -                   
 *
 ****************************************************************************************************
 * Project:      Graduaatsproef Afstandbestuurd Wapen Systeem (Airsoft Demonstrator)
 * Context:      Marinebasis Zeebrugge - Demonstratiemodel voor evenementen/beurzen
 * Author:       D'Haenens Jorrit
 * Date:         04-29-2025
 * Version:      1.0
 *
 * Platform:     Arduino Mega 2560
 * Purpose:      Deze code draait op de Arduino Mega en is verantwoordelijk voor:
 *               - Het ontvangen van besturingscommando's van een ESP32 (via Serial2).
 *               - Het aansturen van twee stappenmotoren voor pan (horizontaal) en
 *                 tilt (verticaal) beweging.
 *               - Het uitlezen van limietschakelaars om bewegingsbereik te beperken.
 *               - Het aansturen van het afvuurmechanisme (magneet/solenoid).
 *               - Het beheren van verschillende vuurmodi (Single, Burst, Auto).
 *               - (Optioneel) Het bijwerken van een OLED display met statusinformatie.
 *
 * Hardware:     - Arduino Mega 2560
 *               - 2x Stappenmotor + Driver (bv. DRV8825, A4988)
 *               - 4x Limietschakelaars
 *               - 1x Relais/MOSFET voor aansturing magneet/solenoid (Airsoft trigger)
 *               - OLED Display SSD1306 (I2C)
 *               - Communicatie met ESP32 via Serial2 (TX2/RX2)
 *
 * !! DISCLAIMER !!
 * Dit systeem is ontwikkeld als een NIET-LETHAAL demonstratiemodel ("speelgoed")
 * met een airsoft-apparaat. Het is uitsluitend bedoeld voor educatieve en
 * representatieve doeleinden tijdens gecontroleerde evenementen en beurzen.
 * Dit is GEEN operationeel wapensysteem. Behandel het airsoft-apparaat altijd
 * met de nodige veiligheidsmaatregelen.
 ************************************************************************************
 *
 * --- FUNCTIONALITEIT OVERZICHT ---
 *
 * --- Geïmplementeerd ---
 *   * Motor-aangedreven beweging: Pan (Rotatie, Motor 1) & Tilt (Elevatie, Motor 2).
 *   * Besturing via PS4 Controller (Bluetooth naar ESP32, serieel naar Mega).
 *   * Joystick Bediening:
 *      - Linker Stick Y (l3Y) -> Pan (Rotatie).
 *      - Rechter Stick Y (r3Y) -> Tilt (Elevatie).
 *   * Vuurmechanisme (Elektromagneet/Solenoid, pin MAGNET_PIN):
 *      - Bediening via L2 + R2 ("Two-man rule" / veiligheidsgrendel).
 *   * Vuurmodi (L1 knop):
 *      - Wisselt tussen SINGLE, BURST, AUTO (beïnvloedt pulsduur magneet).
 *   * Laser Pointer / Extra Output (pin R1_PIN):
 *      - Aan/Uit schakelen met R1 knop.
 *   * Fail-Safe: Elektromagneet lost bij spanningsuitval (voorkomt vastzittende trigger).
 *   * Eindschakelaars (Hardware): Functioneel, fysiek begrenzen van beweging.
 *
 * --- Geïmplementeerd (Kalibratie/Fine-tuning Nodig) ---
 *   * Software Limieten:
 *      - Bewegingsbereik softwarematig beperkt tot Pan ~-45°/+45°, Tilt ~-20°/+65°.
 *      - Vereist nauwkeurige kalibratie van STEPS_PER_DEGREE1/2 om graden correct te laten corresponderen met fysieke beweging (rekening houdend met overbrenging).
 *   * Homing Functie (Share knop):
 *      - Stuurt Pan & Tilt terug naar getelde 0° positie. Magneet wordt gedeactiveerd.
 *      - Nauwkeurigheid afhankelijk van startpositie en geen gemiste stappen.
 *   * Veiligheidszone Pan:
 *      - Vuren softwarematig geblokkeerd binnen +/- 5° rotatie vanaf getelde middenpositie (vereist correcte kalibratie).
 *
 * --- Nog Niet Geïmplementeerd / Toekomstige Uitbreiding ---
 *   * Snelheidsregeling: Implementeren van specifieke doelsnelheid (~20°/sec) voor Pan & Tilt. Huidige snelheid is afhankelijk van puls timing (delayMicroseconds).
 *   * Acceleratie/Deceleratie (Ramping): Soepel starten en stoppen voor motorbewegingen om schokken te verminderen en nauwkeurigheid te verhogen.
 *   * OLED Display Output: Gedetailleerde info (precieze hoeken, sensor statussen, etc.).
 *   * Alternatieve Bediening: Mogelijkheid tot aansturing via HC-05 Bluetooth module (als alternatief/aanvulling op PS4/ESP32).
 *
 ***********************************************************************************/

// ==================================================================================
//                                     CODE START
// ==================================================================================
 
 #include <Wire.h>      // Inclusief de bibliotheek voor I2C communicatie (gebruikt door U8g2)
 #include <U8g2lib.h>   // Inclusief de bibliotheek voor het aansturen van monochrome displays

 // OLED display configuratie: SSD1306 type, 128x64 pixels, geen frame buffer rotatie,
 // gebruikmakend van hardware I2C (SDA/SCL pinnen van de Arduino Mega - pin 20/21)
 U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

 // === Pin Definities Motor 1 (Bijv. Horizontale as / Pan) ===
 const int PUL1 = 7;   // Pulse (STEP) pin voor stappenmotor driver 1
 const int DIR1 = 6;   // Direction pin voor stappenmotor driver 1
 const int ALM1 = 5;   // Alarm/Enable pin voor stappenmotor driver 1 (LOW = meestal enabled)
 const int LIMIT1_A = 2; // Limietschakelaar A (bijv. links/min) voor motor 1 (Interrupt pin)
 const int LIMIT1_B = 3; // Limietschakelaar B (bijv. rechts/max) voor motor 1 (Interrupt pin)

 // === Pin Definities Motor 2 (Bijv. Verticale as / Tilt) ===
 const int PUL2 = 10;  // Pulse (STEP) pin voor stappenmotor driver 2
 const int DIR2 = 9;   // Direction pin voor stappenmotor driver 2
 const int ALM2 = 8;   // Alarm/Enable pin voor stappenmotor driver 2 (LOW = meestal enabled)
 const int LIMIT2_A = 4; // Limietschakelaar A (bijv. omlaag/min) voor motor 2
 const int LIMIT2_B = 11; // Limietschakelaar B (bijv. omhoog/max) voor motor 2

 // === Andere Pin Definities ===
 #define R1_PIN 23      // Digitale output pin, geschakeld door R1 knop
 #define MAGNET_PIN 25  // Digitale output pin voor het aansturen van de magneet (vuurmechanisme)

 // === Vuur Modus Configuratie ===
 enum FiringMode { SINGLE, BURST, AUTO }; // Definieert de beschikbare vuurmodi
 FiringMode firingMode = SINGLE;          // Huidige actieve vuurmodus, start met SINGLE
 const char* firingModeNames[] = {"Single", "Burst", "Auto"}; // Namen voor weergave/debug

 // === Controller Input & Status Variabelen ===
 int l3Y = 0;           // Waarde Y-as Linker Stick (ontvangen van ESP32)
 int r3Y = 0;           // Waarde Y-as Rechter Stick (ontvangen van ESP32)
 int prevR1 = 0;        // Vorige status van R1 knop (voor edge detection)
 int pin23State = LOW;  // Huidige status van de R1_PIN output
 int lastR1 = 0;        // Meest recente status R1 knop (ontvangen)
 int lastL2 = 0;        // Meest recente status L2 trigger (ontvangen)
 int lastR2 = 0;        // Meest recente status R2 trigger (ontvangen)
 int lastL1 = 0;        // Meest recente status L1 knop (ontvangen)
 int lastShare = 0;     // Meest recente status Share knop (ontvangen)
 int l1Value = 0;       // Buffer voor L1 waarde uit bericht
 const int DEADZONE = 20; // Dode zone voor de joysticks om kleine afwijkingen te negeren

 // === Vuurmechanisme Controle Variabelen ===
 bool firingActive = false;             // Geeft aan of het vuurmechanisme momenteel actief is (bv. tijdens burst)
 unsigned long firingStartTime = 0;    // Tijdstip waarop het vuren begon (voor timing burst/single shot)
 bool prevL2R2 = false;                 // Vorige gecombineerde status van L2 en R2 (voor edge detection)

 // === Positie Tracking voor Stappenmotoren ===
 long position1 = 0;  // Huidige positie van motor 1 (in stappen, relatief t.o.v. start/nul)
 long position2 = 0;  // Huidige positie van motor 2 (in stappen, relatief t.o.v. start/nul)
 // BELANGRIJK: Kalibreer deze waarden nauwkeurig!
 const float STEPS_PER_DEGREE1 = 10.0; // Aantal stappen per graad rotatie voor motor 1
 const float STEPS_PER_DEGREE2 = 10.0; // Aantal stappen per graad rotatie voor motor 2

 /**
  * @brief Setup functie: Initialiseert hardware en variabelen bij opstarten.
  */
 void setup() {
  // Start Seriële communicatie voor debug output op de Serial Monitor
  Serial.begin(115200);
  // Start Seriële communicatie op Serial2 voor data ontvangst van de ESP32
  Serial2.begin(115200);

  // === Configureer Pinnen Motor 1 ===
  pinMode(PUL1, OUTPUT);       // Pulse pin is output
  pinMode(DIR1, OUTPUT);       // Direction pin is output
  pinMode(ALM1, OUTPUT);       // Alarm/Enable pin is output
  pinMode(LIMIT1_A, INPUT_PULLUP); // Limietschakelaar A is input met interne pull-up weerstand (LOW = actief)
  pinMode(LIMIT1_B, INPUT_PULLUP); // Limietschakelaar B is input met interne pull-up weerstand (LOW = actief)

  // === Configureer Pinnen Motor 2 ===
  pinMode(PUL2, OUTPUT);       // Pulse pin is output
  pinMode(DIR2, OUTPUT);       // Direction pin is output
  pinMode(ALM2, OUTPUT);       // Alarm/Enable pin is output
  pinMode(LIMIT2_A, INPUT_PULLUP); // Limietschakelaar A is input met interne pull-up weerstand (LOW = actief)
  pinMode(LIMIT2_B, INPUT_PULLUP); // Limietschakelaar B is input met interne pull-up weerstand (LOW = actief)

  // === Configureer Overige Pinnen ===
  pinMode(R1_PIN, OUTPUT);     // R1 output pin
  pinMode(MAGNET_PIN, OUTPUT); // Magneet output pin

  // === Initialiseer Output Pinnen ===
  digitalWrite(ALM1, LOW);     // Activeer/Enable motor driver 1 (ervan uitgaande dat LOW enable is)
  digitalWrite(ALM2, LOW);     // Activeer/Enable motor driver 2 (ervan uitgaande dat LOW enable is)
  digitalWrite(DIR1, LOW);     // Stel een initiële richting in voor motor 1
  digitalWrite(DIR2, HIGH);    // Stel een initiële richting in voor motor 2 (tegengesteld aan 1 als voorbeeld)
  digitalWrite(R1_PIN, pin23State); // Zet R1_PIN naar de initiële staat (LOW)
  digitalWrite(MAGNET_PIN, LOW); // Zorg dat de magneet uit staat bij opstarten

  // Initialiseer het OLED display
  u8g2.begin();

  // Meld dat het systeem klaar is op de Serial Monitor
  Serial.println("System ready.");
 }

 /**
  * @brief Hoofd loop: Verwerkt inkomende data, stuurt motoren aan, regelt vuurmechanisme.
  */
 void loop() {
  // Buffer voor het opvangen van inkomende data van Serial2 (ESP32)
  static String inputString = ""; // Static om de inhoud tussen loop iteraties te behouden

  // Verwerk alle beschikbare karakters van Serial2
  while (Serial2.available()) {
    char inChar = (char)Serial2.read(); // Lees één karakter
    // Als het karakter een newline is, is het einde van het bericht bereikt
    if (inChar == '\n') {
      handleMessage(inputString); // Verwerk het complete bericht
      inputString = "";           // Leeg de buffer voor het volgende bericht
    } else {
      inputString += inChar;      // Voeg het karakter toe aan de buffer
    }
  }

  // Lees de status van de limietschakelaars
  // INPUT_PULLUP betekent dat de pin HIGH is tenzij de schakelaar wordt ingedrukt (verbonden met GND), dan LOW.
  bool limitA1 = digitalRead(LIMIT1_A) == LOW; // True als limietsch. A motor 1 actief is
  bool limitB1 = digitalRead(LIMIT1_B) == LOW; // True als limietsch. B motor 1 actief is
  bool limitA2 = digitalRead(LIMIT2_A) == LOW; // True als limietsch. A motor 2 actief is
  bool limitB2 = digitalRead(LIMIT2_B) == LOW; // True als limietsch. B motor 2 actief is

  // === Share Knop Logica (Terug naar Nulpositie?) ===
  static bool shareActive = false; // Houdt bij of de 'terugkeer' modus actief is
  // Activeer de modus als de Share knop is ingedrukt (lastShare komt van handleMessage)
  if (lastShare == 1) {
    shareActive = true;
  }

  // Als de 'terugkeer' modus actief is
  if (shareActive) {
    // Beweeg motor 1 terug naar 0 als deze niet op 0 staat
    if (abs(position1) > 0) {
      // Bepaal de richting om terug naar 0 te gaan
      if (position1 > 0) digitalWrite(DIR1, HIGH); // Moet tegen de telrichting in
      else digitalWrite(DIR1, LOW);               // Moet met de telrichting mee
      // Genereer een puls
      digitalWrite(PUL1, HIGH);
      delayMicroseconds(30); // Puls duur (pas aan indien nodig)
      digitalWrite(PUL1, LOW);
      delayMicroseconds(50); // Pauze tussen pulsen (pas aan indien nodig)
      // Werk de positie bij op basis van de *werkelijke* bewegingsrichting
      position1 += (digitalRead(DIR1) == LOW) ? 1 : -1; // Pas op: richting kan anders zijn dan verwacht
    }
    // Beweeg motor 2 terug naar 0 als deze niet op 0 staat
    if (abs(position2) > 0) {
      // Bepaal de richting om terug naar 0 te gaan
      if (position2 > 0) digitalWrite(DIR2, HIGH); // Moet tegen de telrichting in
      else digitalWrite(DIR2, LOW);               // Moet met de telrichting mee
      // Genereer een puls
      digitalWrite(PUL2, HIGH);
      delayMicroseconds(30);
      digitalWrite(PUL2, LOW);
      delayMicroseconds(50);
      // Werk de positie bij op basis van de *werkelijke* bewegingsrichting
      position2 += (digitalRead(DIR2) == LOW) ? 1 : -1; // Pas op: richting kan anders zijn dan verwacht
    }
    // Als beide motoren positie 0 hebben bereikt, deactiveer de modus
    if (position1 == 0 && position2 == 0) {
      shareActive = false;
    }
  }
  // === Normale Motorbesturing (als Share niet actief is) ===
  else {
    // --- Motor 1 Besturing (Linker Stick Y) ---
    if (abs(l3Y) > DEADZONE) { // Controleer of de stick buiten de dode zone is
      float motor1Deg = position1 / STEPS_PER_DEGREE1; // Bereken huidige hoek motor 1

      // Beweeg motor 1 "omhoog" (negatieve Y, richting HIGH) indien niet bij limiet A en binnen software limiet
      if (l3Y < 0 && !limitA1 && motor1Deg > -45) {
        digitalWrite(DIR1, HIGH); // Stel richting in
        // Genereer puls
        digitalWrite(PUL1, HIGH);
        delayMicroseconds(30);
        digitalWrite(PUL1, LOW);
        delayMicroseconds(50);
        position1--; // Werk positie bij (richting HIGH = negatieve beweging)
      }
      // Beweeg motor 1 "omlaag" (positieve Y, richting LOW) indien niet bij limiet B en binnen software limiet
      if (l3Y > 0 && !limitB1 && motor1Deg < 45) {
        digitalWrite(DIR1, LOW); // Stel richting in
        // Genereer puls
        digitalWrite(PUL1, HIGH);
        delayMicroseconds(30);
        digitalWrite(PUL1, LOW);
        delayMicroseconds(50);
        position1++; // Werk positie bij (richting LOW = positieve beweging)
      }
    }
    // --- Motor 2 Besturing (Rechter Stick Y) ---
    if (abs(r3Y) > DEADZONE) { // Controleer of de stick buiten de dode zone is
      float motor2Deg = position2 / STEPS_PER_DEGREE2; // Bereken huidige hoek motor 2

      // Beweeg motor 2 "omhoog" (negatieve Y, richting HIGH) indien niet bij limiet A en binnen software limiet
      if (r3Y < 0 && !limitA2 && motor2Deg > -20) {
        digitalWrite(DIR2, HIGH); // Stel richting in
        // Genereer puls
        digitalWrite(PUL2, HIGH);
        delayMicroseconds(30);
        digitalWrite(PUL2, LOW);
        delayMicroseconds(50);
        position2--; // Werk positie bij (richting HIGH = negatieve beweging)
      }
      // Beweeg motor 2 "omlaag" (positieve Y, richting LOW) indien niet bij limiet B en binnen software limiet
      if (r3Y > 0 && !limitB2 && motor2Deg < 65) {
        digitalWrite(DIR2, LOW); // Stel richting in
        // Genereer puls
        digitalWrite(PUL2, HIGH);
        delayMicroseconds(30);
        digitalWrite(PUL2, LOW);
        delayMicroseconds(50);
        position2++; // Werk positie bij (richting LOW = positieve beweging)
      }
    }
  } // Einde normale motorbesturing

  // === R1 Knop Logica (Toggle Pin 23) ===
  // Detecteer een neergaande flank (knop was ingedrukt, wordt nu losgelaten)
  if (lastR1 == 0 && prevR1 == 1) {
    pin23State = (pin23State == LOW) ? HIGH : LOW; // Wissel de status van de pin
    digitalWrite(R1_PIN, pin23State);             // Zet de output pin naar de nieuwe status
  }
  prevR1 = lastR1; // Update de vorige status voor de volgende iteratie

  // === Magneet Controle / Vuurmechanisme ===
  // Controleer of beide triggers (L2 en R2) zijn ingedrukt
  bool l2r2 = (lastL2 > 0 && lastR2 > 0); // Gebruik > 0 voor analoge triggers indien nodig, anders == 1

  // --- Vuurmodus Selectie (L1 Knop) ---
  static int prevL1 = 0; // Onthoud vorige L1 status
  // Detecteer een opgaande flank (knop was niet ingedrukt, wordt nu wel ingedrukt)
  if (prevL1 == 0 && l1Value == 1) {
    // Ga naar de volgende vuurmodus (cyclisch: SINGLE -> BURST -> AUTO -> SINGLE)
    firingMode = (FiringMode)((firingMode + 1) % 3); // % 3 zorgt voor wrap-around
    // Print de nieuwe modus naar de Serial Monitor voor debuggen
    Serial.print("Firing mode: "); Serial.println(firingModeNames[firingMode]);
  }
  prevL1 = l1Value; // Update vorige L1 status

  // --- Vuurlogica per Modus ---
  // ** BELANGRIJKE AANNAMES **
  // - Vuren gebeurt alleen als de horizontale motor (Motor 1) meer dan 5 graden van het midden is.
  // - Dit voorkomt mogelijk vuren in een onveilige/ongewenste richting.

  // --- SINGLE Modus ---
  if (firingMode == SINGLE) {
    // Als L2+R2 worden ingedrukt (opgaande flank) EN er niet al gevuurd wordt
    if (l2r2 && !prevL2R2 && !firingActive) {
      // Controleer of motor 1 voldoende gedraaid is
      if (abs(position1 / STEPS_PER_DEGREE1) > 5) {
        firingActive = true;              // Markeer als actief vuren
        firingStartTime = millis();       // Start de timer
        digitalWrite(MAGNET_PIN, HIGH);   // Activeer de magneet
      }
    }
    // Als er gevuurd wordt en de tijd voor een enkel schot (67ms) is verstreken
    if (firingActive && millis() - firingStartTime >= 67) {
      digitalWrite(MAGNET_PIN, LOW);    // Deactiveer de magneet
      firingActive = false;             // Markeer als niet meer actief vuren
    }
    // Als L2+R2 worden losgelaten (veiligheid)
    if (!l2r2) {
      digitalWrite(MAGNET_PIN, LOW);    // Zorg dat magneet uit is
      firingActive = false;             // Reset vuurstatus
    }
  }
  // --- BURST Modus ---
  else if (firingMode == BURST) {
    // Als L2+R2 worden ingedrukt (opgaande flank) EN er niet al gevuurd wordt
    if (l2r2 && !prevL2R2 && !firingActive) {
       // Controleer of motor 1 voldoende gedraaid is
      if (abs(position1 / STEPS_PER_DEGREE1) > 5) {
        firingActive = true;              // Markeer als actief vuren
        firingStartTime = millis();       // Start de timer
        digitalWrite(MAGNET_PIN, HIGH);   // Activeer de magneet
      }
    }
    // Als er gevuurd wordt en de tijd voor de burst (200ms) is verstreken
    if (firingActive && millis() - firingStartTime >= 200) {
      digitalWrite(MAGNET_PIN, LOW);    // Deactiveer de magneet
      firingActive = false;             // Markeer als niet meer actief vuren
    }
    // Als L2+R2 worden losgelaten
    if (!l2r2) {
      digitalWrite(MAGNET_PIN, LOW);    // Zorg dat magneet uit is
      firingActive = false;             // Reset vuurstatus
    }
  }
  // --- AUTO Modus ---
  else if (firingMode == AUTO) {
    // Als L2+R2 ingedrukt zijn
    if (l2r2) {
       // Controleer of motor 1 voldoende gedraaid is
      if (abs(position1 / STEPS_PER_DEGREE1) > 5) {
        digitalWrite(MAGNET_PIN, HIGH); // Houd de magneet actief
      } else {
        digitalWrite(MAGNET_PIN, LOW);  // Schakel uit als motor te dicht bij midden is
      }
    } else {
      // Als L2+R2 worden losgelaten, schakel de magneet uit
      digitalWrite(MAGNET_PIN, LOW);
    }
    // In AUTO mode is 'firingActive' niet nodig voor timing, dus altijd false
    firingActive = false;
  }

  // Update de vorige L2+R2 status voor de volgende iteratie (belangrijk voor flank detectie)
  prevL2R2 = l2r2;

  // Voeg hier eventueel code toe voor het OLED display (u8g2.clearBuffer(), u8g2.draw...(), u8g2.sendBuffer())
  // Bijvoorbeeld: positie, modus, batterij status (indien beschikbaar)
 }

 /**
  * @brief Verwerkt een binnenkomend bericht (CSV string) van de ESP32.
  * @param message De volledige regel data ontvangen via Serial2.
  */
 void handleMessage(const String& message) {
  int field = 0;        // Telt de velden (komma-gescheiden waarden)
  int lastIndex = 0;    // Houdt de startpositie bij van het huidige veld
  // Tijdelijke variabelen om waarden uit het bericht te lezen voordat ze globaal worden opgeslagen
  int r1Value = 0, l2Value = 0, r2Value = 0, l1 = 0, share = 0;

  // Loop door het bericht karakter voor karakter
  for (int i = 0; i < message.length(); i++) {
    // Als een komma wordt gevonden of het einde van het bericht is bereikt
    if (message[i] == ',' || i == message.length() - 1) {
      // Extraheer de waarde van het huidige veld
      // (i == message.length() - 1 ? 1 : 0) zorgt dat het laatste karakter meegenomen wordt
      String valueStr = message.substring(lastIndex, i + (i == message.length() - 1 ? 1 : 0));

      // Wijs de waarde toe aan de juiste variabele op basis van het veldnummer (field index)
      // Deze indices moeten overeenkomen met de volgorde in de sprintf op de ESP32!
      // 0: LStickX (niet gebruikt)
      // 1: LStickY -> l3Y
      // 2: RStickX (niet gebruikt)
      // 3: RStickY -> r3Y
      // ... (andere knoppen niet direct gebruikt in deze functie)
      // 12: L1 -> l1
      // 13: R1 -> r1Value
      // 14: L2 -> l2Value
      // 15: R2 -> r2Value
      // 16: Share -> share
      // ... (rest niet gebruikt in deze functie)
      if (field == 1) l3Y = valueStr.toInt(); // Linker Stick Y
      if (field == 3) r3Y = valueStr.toInt(); // Rechter Stick Y
      if (field == 12) l1 = valueStr.toInt();  // L1 Knop
      if (field == 13) r1Value = valueStr.toInt(); // R1 Knop
      if (field == 14) l2Value = valueStr.toInt(); // L2 Trigger
      if (field == 15) r2Value = valueStr.toInt(); // R2 Trigger
      if (field == 16) share = valueStr.toInt(); // Share Knop

      field++; // Ga naar het volgende veld
      lastIndex = i + 1; // Update de startpositie voor het volgende veld
    }
  }

  // Update de globale 'last' variabelen met de zojuist gelezen waarden
  // Deze worden in de loop() gebruikt voor edge detection en status checks
  lastR1 = r1Value;
  lastL2 = l2Value;
  lastR2 = r2Value;
  l1Value = l1;      // Sla de L1 waarde op voor modus wisseling in de loop
  lastShare = share;
 }
