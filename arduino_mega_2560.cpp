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
  * Date:         06-06-2025
  * Version:      1.3 
  *
  * Platform:     Arduino Mega 2560
  * Purpose:      Deze code draait op de Arduino Mega en is verantwoordelijk voor:
  *               - Het ontvangen van besturingscommando's van een ESP32 (via Serial2).
  *               - Het uitvoeren van een homing-sequentie voor de pan-as bij opstarten
  *                 of op commando (Share knop).
  *               - Het aansturen van twee servomotoren voor pan (horizontaal) en
  *                 tilt (verticaal) beweging, alleen na succesvolle homing.
  *               - Het uitlezen van limietschakelaars om bewegingsbereik te beperken.
  *               - Het aansturen van het afvuurmechanisme (solenoid).
  *               - Het beheren van verschillende vuurmodi (Single, Burst, Auto).
  *               - Het bijwerken van een OLED display met diagnostische informatie.
  *
  * Hardware:     - Arduino Mega 2560
  *				  - ESP32-NodeMCU
  *               - 2x servomotoren (ISV57T)
  *               - 5x Limietschakelaars (2x pan, 2x tilt, 1x pan-homing)
  *               - 1x MOSFET (XY-MOS) voor aansturing solenoid (JF-0530B)
  *               - 1x KY-008 laser pointer
  *               - 1x HC-05 Bluetooth module voor app besturing(werking niet geïmplementeerd in code)
  *               - OLED Display SH1106 (128x64, I2C)
  *               - Communicatie met ESP32 (PS4 Controller Interface)
  *
  * BELANGRIJK!:  Homing MOET voltooid zijn voordat normale bediening mogelijk is.
  *               De Share-knop op de PS4 controller start de homing-sequentie.
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
 *   * Laser Pointer (pin R1_PIN):
 *      - Aan/Uit schakelen met R1 knop.
 *   * Fail-Safe: Elektromagneet lost bij spanningsuitval (voorkomt vastzittende trigger).
 *   * Eindschakelaars (Hardware): Functioneel, fysiek begrenzen van beweging.
 *   * OLED Display Output: Gedetailleerde info (precieze hoeken, sensor statussen, etc.).
 *   * Homing Functie (Share knop):
 *      - Stuurt Pan & Tilt terug naar getelde 0° positie. Magneet wordt gedeactiveerd.
 *      - Nauwkeurigheid afhankelijk van startpositie en geen gemiste stappen.
 *
 * --- Nog Niet Geïmplementeerd / Toekomstige Uitbreiding ---
 *   * Snelheidsregeling: Implementeren van specifieke doelsnelheid (~20°/sec) voor Pan & Tilt. Huidige snelheid is afhankelijk van puls timing (delayMicroseconds).
 *   * Acceleratie/Deceleratie (Ramping): Soepel starten en stoppen voor motorbewegingen om schokken te verminderen en nauwkeurigheid te verhogen.
 *   * Alternatieve Bediening: Mogelijkheid tot aansturing via HC-05 Bluetooth module (als alternatief/aanvulling op PS4/ESP32).
 *   * Veiligheidszone Pan:
 *     - Vuren softwarematig geblokkeerd binnen +/- 5° rotatie vanaf getelde middenpositie (vereist correcte kalibratie).
 *
 ***********************************************************************************/
 
 #include <Wire.h>      // Importeer de bibliotheek voor I2C communicatie (nodig voor U8g2).
 #include <U8g2lib.h>   // Importeer de bibliotheek voor het aansturen van monochrome displays.

 // === OLED Display Configuratie ===
 // Initialiseer het U8g2 display object voor een SH1106 128x64 OLED display.
 // U8G2_R0: Geen rotatie.
 // _F_: Full buffer mode (vereist meer RAM, maar flexibeler).
 // _HW_I2C: Gebruik hardware I2C pinnen van de Arduino Mega (SDA=pin 20, SCL=pin 21).
 // U8X8_PIN_NONE: Geen aparte reset pin gedefinieerd/gebruikt.
 U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

 // === Pin Definities Motor 1 (Pan / Horizontale As) ===
 const int PUL1 = 7;   // Pulse (STEP) pin voor servomotor driver 1.
 const int DIR1 = 6;   // Direction (DIR) pin voor servomotor driver 1.
 const int ALM1 = 5;   // Alarm/Enable (ALM/EN) pin voor servomotor driver 1.
 const int LIMIT1_A = 2; // Input pin voor limietschakelaar A (uiterst links) voor servomotor 1.
 const int LIMIT1_B = 3; // Input pin voor limietschakelaar B (uiterst rechts) voor servomotor 1.
 const int LIMIT3_C = 12; // Input pin voor homing-schakelaar C (middenpositie / referentie) voor servomotor 1.

 // === Pin Definities Motor 2 (Tilt / Verticale As) ===
 const int PUL2 = 10;  // Pulse (STEP) pin voor servomotor driver 2.
 const int DIR2 = 9;   // Direction (DIR) pin voor servomotor driver 2.
 const int ALM2 = 8;   // Alarm/Enable (ALM/EN) pin voor servomotor driver 2.
 const int LIMIT2_A = 4;  // Input pin voor limietschakelaar A (uiterst omlaag) voor servomotor 2.
 const int LIMIT2_B = 11; // Input pin voor limietschakelaar B (uiterst omhoog) voor servomotor 2.

 // === Andere Pin Definities ===
 #define R1_PIN 23      // Digitale output pin, geschakeld door R1 knop (laser).
 #define MAGNET_PIN 25  // Digitale output pin voor het aansturen van de elektromagneet (vuurmechanisme).

 // === Vuur Modus Configuratie ===
 enum FiringMode { SINGLE, BURST, AUTO }; // Definieert de beschikbare vuurmodi.
 FiringMode firingMode = SINGLE;          // Huidige actieve vuurmodus, start met SINGLE.
 const char* firingModeNames[] = {"Single", "Burst", "Auto"}; // Namen voor weergave/debug.

 // === Controller Input & Status Variabelen ===
 int l3Y = 0;           // Waarde Y-as Linker Stick (voor pan, van ESP32).
 int r3Y = 0;           // Waarde Y-as Rechter Stick (voor tilt, van ESP32).
 int prevR1 = 0;        // Vorige status van R1 knop (voor flankdetectie).
 int pin23State = LOW;  // Huidige status van de R1_PIN output (laser).
 int lastR1 = 0;        // Meest recente status R1 knop (ontvangen).
 int lastL2 = 0;        // Meest recente status L2 trigger (ontvangen, ruwe waarde 0-255).
 int lastR2 = 0;        // Meest recente status R2 trigger (ontvangen, ruwe waarde 0-255).
 int lastL1 = 0;        // Meest recente status L1 knop (ontvangen).
 int lastShare = 0;     // Meest recente status Share knop (ontvangen, start homing).
 int l1Value = 0;       // Buffer voor L1 waarde uit bericht (voor modus wissel).
 const int DEADZONE = 20; // Dode zone voor de joysticks om kleine afwijkingen te negeren.

 // === Vuurmechanisme Controle Variabelen ===
 bool firingActive = false;             // Geeft aan of het vuurmechanisme momenteel actief is (timing puls).
 unsigned long firingStartTime = 0;    // Tijdstip waarop het vuren begon (voor timing puls).
 bool prevL2R2 = false;                 // Vorige gecombineerde status van L2 en R2 (voor flankdetectie).

 // === Positie Tracking voor servomotoren ===
 long position1 = 0;  // Huidige positie van servomotor 1 (pan) in stappen, relatief t.o.v. homing positie.
 long position2 = 0;  // Huidige positie van servomotor 2 (tilt) in stappen.
 const float STEPS_PER_DEGREE1 = 10.0; // Aantal stappen per graad rotatie voor servomotor 1.
 const float STEPS_PER_DEGREE2 = 10.0; // Aantal stappen per graad rotatie voor servomotor 2.

 // === Homing Variabelen & Constanten ===
 const long HOME_POSITION = 0; // Definieert de 'thuis' positie in stappen (na raken LIMIT3_C).
 const long LIMIT_A_POSITION = -5000; // Geschatte positie (in stappen) van limietschakelaar A relatief aan thuis.
 bool isHomed = false;         // Vlag die aangeeft of de homing-procedure succesvol is voltooid.
 unsigned long homingTimeout = 30000; // Maximale tijd (in ms) voor de homing-sequentie (30 seconden).

 // === Homing States ===
 enum HomingState {
   HOMING_IDLE,       // Geen homing actief, of homing voltooid.
   HOMING_TO_A,       // Fase 1: Beweeg motor 1 naar limietschakelaar A.
   HOMING_TO_C,       // Fase 2: Beweeg motor 1 van A naar homing-schakelaar C.
   HOMING_COMPLETE    // Homing is succesvol afgerond.
 };

 // === OLED Display Variabelen ===
 unsigned long lastDisplayUpdate = 0;             // Tijdstip van de laatste display update.
 const unsigned long DISPLAY_UPDATE_INTERVAL = 2000; // Update het display elke 2 seconden (tenzij geforceerd).
 HomingState currentHomingState = HOMING_IDLE;    // Huidige homing staat.
 bool displayNeedsUpdate = false;                  // Vlag om een onmiddellijke display update te forceren.
 bool motorActive = false;                       // Vlag om te bepalen of motoren recent bewogen zijn (voor display update timing).

 // Variabelen om de vorige status van limietschakelaars bij te houden voor display updates.
 bool prevLimitA1_display = false;
 bool prevLimitB1_display = false;
 bool prevLimitA2_display = false;
 bool prevLimitB2_display = false;
 bool prevLimitC3_display = false;


 // === Functie Prototypes (declaraties) ===
 // Dit stelt de compiler in staat om functies te vinden, zelfs als ze later in de code gedefinieerd zijn.
 void stepMotor(int motor, int delayMicros = 10); // Genereert een stap voor de opgegeven motor.
 void handleMessage(const String& message);       // Verwerkt inkomende berichten van de ESP32.
 void handleFiringLogic();                        // Behandelt de logica voor het afvuren.
 void startHomingSequence();                      // Start de homing-procedure.
 bool performHomingToA(bool limitA1);             // Uitvoeren van homing fase 1 (naar limiet A).
 bool performHomingToC(bool limitC3);             // Uitvoeren van homing fase 2 (naar home C).
 void updateDisplay();                            // Werkt het OLED display bij met statusinformatie.
 void initDisplay();                              // Initialiseert het OLED display bij opstarten.

 /**
  * @brief Setup functie: Wordt eenmalig uitgevoerd bij het opstarten van de Arduino.
  */
 void setup() {
   // Start Seriële communicatie voor debug output op de Serial Monitor (USB).
   Serial.begin(115200);
   // Start Seriële communicatie op Serial2 voor data ontvangst van de ESP32.
   Serial2.begin(115200);

   // Initialiseer het OLED display en toon een opstartbericht.
   initDisplay();

   // Configureer pinnen voor servomotor 1 (pan) als OUTPUT en INPUT_PULLUP.
   pinMode(PUL1, OUTPUT);
   pinMode(DIR1, OUTPUT);
   pinMode(ALM1, OUTPUT);
   pinMode(LIMIT1_A, INPUT_PULLUP); // INPUT_PULLUP: interne weerstand naar VCC, schakelaar trekt naar GND.
   pinMode(LIMIT1_B, INPUT_PULLUP);
   pinMode(LIMIT3_C, INPUT_PULLUP); // Homing-schakelaar.

   // Configureer pinnen voor servomotor 2 (tilt) als OUTPUT en INPUT_PULLUP.
   pinMode(PUL2, OUTPUT);
   pinMode(DIR2, OUTPUT);
   pinMode(ALM2, OUTPUT);
   pinMode(LIMIT2_A, INPUT_PULLUP);
   pinMode(LIMIT2_B, INPUT_PULLUP);

   // Configureer overige output pinnen.
   pinMode(R1_PIN, OUTPUT);     // Pin voor laser.
   pinMode(MAGNET_PIN, OUTPUT); // Pin voor elektromagneet.

   // Stel de initiële status van de output pinnen in.
   digitalWrite(ALM1, LOW);     // Activeer servomotor driver 1 (LOW = enable).
   digitalWrite(ALM2, LOW);     // Activeer servomotor driver 2.
   digitalWrite(DIR1, LOW);     // Stel een initiële richting in voor servomotor 1.
   digitalWrite(DIR2, HIGH);    // Stel een initiële richting in voor servomotor 2.
   digitalWrite(R1_PIN, pin23State); // Zet laser naar de initiële staat (LOW/uit).
   digitalWrite(MAGNET_PIN, LOW); // Zorg dat de magneet uit staat bij opstarten.

   // Berichten naar de Serial Monitor.
   Serial.println("System ready. Press SHARE to start homing sequence.");
   Serial.println("IMPORTANT: Homing must be completed before normal operation!");
   
   // Initialiseer de 'vorige' status van limietschakelaars voor de display logica.
   prevLimitA1_display = (digitalRead(LIMIT1_A) == LOW);
   prevLimitB1_display = (digitalRead(LIMIT1_B) == LOW);
   prevLimitA2_display = (digitalRead(LIMIT2_A) == LOW);
   prevLimitB2_display = (digitalRead(LIMIT2_B) == LOW);
   prevLimitC3_display = (digitalRead(LIMIT3_C) == LOW);

   // Update het display direct na de setup.
   updateDisplay();
 } // Einde setup()

 /**
  * @brief Hoofd loop: Wordt continu herhaald na setup().
  */
 void loop() {
   // Buffer voor het opvangen van inkomende data van Serial2 (ESP32).
   static String inputString = ""; // 'static' om inhoud te behouden tussen iteraties.
   // Verwerk alle beschikbare karakters van Serial2.
   while (Serial2.available()) {
     char inChar = (char)Serial2.read(); // Lees één karakter.
     if (inChar == '\n') { // Einde van bericht (newline karakter).
       handleMessage(inputString); // Verwerk het complete bericht.
       inputString = "";           // Leeg de buffer.
     } else {
       // Voeg karakter toe, maar voorkom buffer overflow.
       if (inputString.length() < 199) { // Max lengte CSV string is ca. 200
          inputString += inChar;
       }
     }
   }

   // Lees de huidige status van alle limietschakelaars.
   // LOW betekent dat de schakelaar is ingedrukt (actief).
   bool limitA1 = digitalRead(LIMIT1_A) == LOW;
   bool limitB1 = digitalRead(LIMIT1_B) == LOW;
   bool limitA2 = digitalRead(LIMIT2_A) == LOW;
   bool limitB2 = digitalRead(LIMIT2_B) == LOW;
   bool limitC3 = digitalRead(LIMIT3_C) == LOW; // Homing-schakelaar.

   // Controleer of de status van een limietschakelaar is veranderd sinds de laatste check.
   // Indien ja, zet de vlag om het display te updaten.
   if (limitA1 != prevLimitA1_display || 
       limitB1 != prevLimitB1_display ||
       limitA2 != prevLimitA2_display ||
       limitB2 != prevLimitB2_display ||
       limitC3 != prevLimitC3_display) {
     displayNeedsUpdate = true; // Forceer een display update.
     // Update de 'vorige' statussen voor de volgende vergelijking.
     prevLimitA1_display = limitA1;
     prevLimitB1_display = limitB1;
     prevLimitA2_display = limitA2;
     prevLimitB2_display = limitB2;
     prevLimitC3_display = limitC3;
   }

   // === Homing State ===
   static HomingState homingState = HOMING_IDLE; // Huidige staat van de homing-procedure.
   static bool prevShareButtonState = false;      // Vorige status van de Share knop.
   static unsigned long homingStartTime = 0;     // Tijdstip waarop homing is gestart (voor timeout).

   // Lees de huidige status van de Share knop (uit de laatst ontvangen PS4 data).
   bool currentShareButtonState = (lastShare == 1);

   // Detecteer een opgaande flank van de Share knop EN als er geen homing bezig is.
   if (currentShareButtonState && !prevShareButtonState && homingState == HOMING_IDLE) {
     isHomed = false; // Markeer systeem als niet-gehomed.
     startHomingSequence();     // Initialiseer de homing-procedure.
     homingState = HOMING_TO_A; // Ga naar de eerste homing-staat.
     homingStartTime = millis();  // Start de timeout timer.
     Serial.println("Share button pressed - starting homing sequence");
     displayNeedsUpdate = true;  // Update display met nieuwe homing status.
   }
   // Update de vorige status van de Share knop voor de volgende iteratie.
   prevShareButtonState = currentShareButtonState;

   // Update de globale homing staat variabele (voor display).
   currentHomingState = homingState; 

   // Voer acties uit gebaseerd op de huidige homing-staat.
   switch (homingState) {
     case HOMING_TO_A: // Fase 1: Beweeg naar limiet A.
       // performHomingToA() geeft true terug als limiet A bereikt is.
       if (performHomingToA(limitA1)) {
         homingState = HOMING_TO_C; // Ga naar de volgende staat.
         Serial.println("Phase 1 complete: Reached limit A (pan left)");
         Serial.println("Phase 2: Moving to home position C (pan center)...");
         delay(200); // Korte pauze.
         displayNeedsUpdate = true;
       }
       break;

     case HOMING_TO_C: // Fase 2: Beweeg van A naar homing-schakelaar C.
       // performHomingToC() geeft true terug als homing-schakelaar C bereikt is.
       if (performHomingToC(limitC3)) {
         homingState = HOMING_COMPLETE; // Homing is bijna voltooid.
         Serial.println("HOMING COMPLETE: Motor positioned at home C (pan center)");
         Serial.println("Normal operation now available.");
         displayNeedsUpdate = true;
       }
       break;

     case HOMING_COMPLETE: // Homing voltooid.
       homingState = HOMING_IDLE; // Ga terug naar de IDLE staat.
       isHomed = true;            // Markeer systeem als succesvol gehomed.
       displayNeedsUpdate = true;
       break;
     case HOMING_IDLE: // Geen homing actie nodig.
       break;
   }

   // Controleer op timeout tijdens de homing-procedure.
   if (homingState != HOMING_IDLE && homingState != HOMING_COMPLETE && millis() - homingStartTime > homingTimeout) {
     Serial.println("HOMING TIMEOUT! Check limit switches and servomotor connections.");
     homingState = HOMING_IDLE; // Stop de homing poging.
     isHomed = false;           // Systeem is niet gehomed.
     displayNeedsUpdate = true;
   }

   // === Normale Bediening (alleen als systeem gehomed is en geen homing bezig is) ===
   if (isHomed && homingState == HOMING_IDLE) {
     motorActive = false; // Reset servomotor activiteit vlag voor deze iteratie.
     
     // --- servomotor 1 (Pan) Besturing ---
     if (abs(l3Y) > DEADZONE) { // Buiten dode zone.
       motorActive = true; // servomotor 1 is actief.
       // Beweeg naar links (negatief Y) indien niet bij limiet A.
       if (l3Y < 0 && !limitA1) {
         digitalWrite(DIR1, HIGH); // Stel richting in.
         stepMotor(1);             // Zet één stap.
         position1--;              // Werk positie teller bij.
       }
       // Beweeg naar rechts (positief Y) indien niet bij limiet B.
       if (l3Y > 0 && !limitB1) {
         digitalWrite(DIR1, LOW);  // Stel richting in.
         stepMotor(1);             // Zet één stap.
         position1++;              // Werk positie teller bij.
       }
     }

     // --- servomotor 2 (Tilt) Besturing ---
     if (abs(r3Y) > DEADZONE) { // Buiten dode zone.
       motorActive = true; // servomotor 2 is actief.
       // Beweeg omhoog (negatief Y) indien niet bij limiet A (tilt boven).
       if (r3Y < 0 && !limitA2) { // LIMIT2_A is hier de BOVENSTE limiet.
         digitalWrite(DIR2, HIGH); // Stel richting in.
         stepMotor(2);             // Zet één stap.
         position2--;              // Werk positie teller bij.
       }
       // Beweeg omlaag (positief Y) indien niet bij limiet B (tilt onder).
       if (r3Y > 0 && !limitB2) { // LIMIT2_B is hier de ONDERSTE limiet.
         digitalWrite(DIR2, LOW);  // Stel richting in.
         stepMotor(2);             // Zet één stap.
         position2++;              // Werk positie teller bij.
       }
     }

     // --- R1 Knop Logica (Laser Toggle) ---
     // Detecteer neergaande flank: R1 was ingedrukt (1) en is nu losgelaten (0).
     if (lastR1 == 0 && prevR1 == 1) {
       pin23State = (pin23State == LOW) ? HIGH : LOW; // Wissel status.
       digitalWrite(R1_PIN, pin23State);             // Zet pin.
       displayNeedsUpdate = true; // Update display met nieuwe laser status.
     }
   } else if (!isHomed && homingState == HOMING_IDLE) {
     // Als systeem niet gehomed is en geen homing bezig, geef periodieke waarschuwing.
     static unsigned long lastWarning = 0;
     if (millis() - lastWarning > 5000) { // Elke 5 seconden.
       Serial.println("WARNING: System not homed! Press SHARE to start homing.");
       lastWarning = millis();
     }
   }

   // Update vorige R1 status voor volgende iteratie.
   prevR1 = lastR1;

   // === Vuurlogica (alleen als systeem gehomed is) ===
   if (isHomed) {
     handleFiringLogic(); // Roep de aparte functie aan voor vuurlogica.
   }

   // === Display Update Logica ===
   // Update het display als er geen motor- of vuuractie bezig is.
   // EN als de update interval verstreken is OF een update geforceerd is.
   if (!motorActive && !firingActive && homingState == HOMING_IDLE) {
     if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL || displayNeedsUpdate) {
       updateDisplay(); // Roep de display update functie aan.
       lastDisplayUpdate = millis(); // Reset de timer.
       displayNeedsUpdate = false;   // Reset de forceer vlag.
     }
   }
 } // Einde loop()

 /**
  * @brief Initialiseert het OLED display en toont een opstartbericht.
  */
 void initDisplay() {
   u8g2.begin(); // Start communicatie met het display.
   u8g2.clearBuffer(); // Wis de interne buffer.
   u8g2.setFont(u8g2_font_6x10_tf); // Stel een lettertype in.
   u8g2.drawStr(0, 15, "Turret System"); // Tekst op scherm.
   u8g2.drawStr(0, 30, "Initializing...");
   u8g2.sendBuffer(); // Stuur buffer naar display.
   delay(1000); // Wacht even zodat het bericht leesbaar is.
 }

 /**
  * @brief Werkt het OLED display bij met actuele statusinformatie.
  *        Wordt alleen aangeroepen als er geen snelle motor/vuuracties zijn.
  */
 void updateDisplay() {
   // Als motoren of vuurmechanisme actief zijn, sla display update over om prioriteit te geven.
   if (motorActive || firingActive) {
       return; 
   }
   
   u8g2.clearBuffer(); // Wis buffer.
   u8g2.setFont(u8g2_font_6x10_tf); // Standaard lettertype voor titel.
   
   // Teken titel en scheidingslijn.
   u8g2.drawStr(0, 10, "TURRET DIAGNOSTICS");
   u8g2.drawLine(0, 12, 128, 12); // Horizontale lijn.
   
   // Homing status.
   String homingStatus = "HOME: ";
   if (!isHomed && currentHomingState == HOMING_IDLE) { // Niet gehomed, geen homing bezig.
       homingStatus += "NOK"; // NOK = Niet Oké.
   } else if (!isHomed) { // Homing bezig.
     switch (currentHomingState) {
       case HOMING_IDLE: homingStatus += "NOK"; break; 
       case HOMING_TO_A: homingStatus += "->A"; break; // Bezig naar limiet A.
       case HOMING_TO_C: homINGStatus += "->C"; break; // Bezig naar homing-schakelaar C.
       case HOMING_COMPLETE: homingStatus += "DONE"; break; // Net voltooid.
     }
   } else { // Succesvol gehomed.
     homingStatus += "OK";
   }
   u8g2.drawStr(0, 24, homingStatus.c_str()); // Teken homing status.
   
   // Vuurmodus.
   String fireStr = "FIRE: " + String(firingModeNames[firingMode]);
   u8g2.drawStr(0, 36, fireStr.c_str()); // Teken vuurmodus.
   
   // Status limietschakelaars (kleiner lettertype voor meer info).
   u8g2.setFont(u8g2_font_5x7_tf); // Kleiner lettertype.
   // Lees huidige status van limietschakelaars (direct voor display).
   bool currentLimitA1_disp = (digitalRead(LIMIT1_A) == LOW);
   bool currentLimitB1_disp = (digitalRead(LIMIT1_B) == LOW);
   bool currentLimitA2_disp = (digitalRead(LIMIT2_A) == LOW);
   bool currentLimitB2_disp = (digitalRead(LIMIT2_B) == LOW);
   bool currentLimitC3_disp = (digitalRead(LIMIT3_C) == LOW); // Homing-schakelaar.
   
   // Toon labels en status (met '*' indien actief).
   u8g2.drawStr(80, 24, "LIMITS:"); // Sectie titel.
   u8g2.drawStr(80, 33, currentLimitA1_disp ? "A1*" : "A1"); // Pan Links.
   u8g2.drawStr(80, 42, currentLimitB1_disp ? "B1*" : "B1"); // Pan Rechts.
   u8g2.drawStr(80, 51, currentLimitC3_disp ? "C3*" : "C3"); // Pan Midden (Home).
   u8g2.drawStr(100, 33, currentLimitA2_disp ? "A2*" : "A2"); // Tilt Boven.
   u8g2.drawStr(100, 42, currentLimitB2_disp ? "B2*" : "B2"); // Tilt Onder.
   
   // Laser status (R1_PIN).
   u8g2.drawStr(80, 60, pin23State == HIGH ? "L:*" : "L:"); // L voor Laser, '*' = aan.
   
   u8g2.sendBuffer(); // Stuur buffer naar display.
 }

 /**
  * @brief Start de homing-sequentie voor servomotor 1 (pan).
  *        Zet de initiële richting en reset de positie teller.
  */
 void startHomingSequence() {
   Serial.println("=== STARTING HOMING SEQUENCE (PAN AXIS) ===");
   Serial.println("Phase 1: Moving to limit A (pan left)...");
   Serial.println("DO NOT INTERRUPT - Homing in progress!");
   position1 = 0; // Reset positie teller (referentie voor deze homing poging).
   digitalWrite(DIR1, HIGH); // Stel richting in om naar limiet A te bewegen (HIGH = links).
 }

 /**
  * @brief Voert fase 1 van de homing uit: beweeg servomotor 1 naar limietschakelaar A.
  * @param limitA1_current De huidige status van limietschakelaar A.
  * @return true als limietschakelaar A is bereikt, anders false.
  */
 bool performHomingToA(bool limitA1_current) {
   if (!limitA1_current) { // Als limiet A nog NIET bereikt is.
     stepMotor(1, 30); // Zet een stap met servomotor 1 (pulsduur 30us).
     // position1 wordt hier NIET bijgewerkt, omdat de exacte positie pas bekend is na raken limiet.
     return false; // Homing fase 1 is nog niet voltooid.
   } else { // Limiet A is bereikt.
     position1 = LIMIT_A_POSITION; // Stel de positie teller in op de gedefinieerde waarde voor limiet A.
     digitalWrite(DIR1, LOW); // Keer de richting om, voor beweging naar homing-schakelaar C.
     return true; // Homing fase 1 is voltooid.
   }
 }

 /**
  * @brief Voert fase 2 van de homing uit: beweeg servomotor 1 van A naar homing-schakelaar C.
  * @param limitC3_current De huidige status van homing-schakelaar C.
  * @return true als homing-schakelaar C is bereikt, anders false.
  */
 bool performHomingToC(bool limitC3_current) {
   if (!limitC3_current) { // Als homing-schakelaar C nog NIET bereikt is.
     stepMotor(1, 50); // Zet een stap.
     position1++;      // Werk positie teller bij (beweging van A naar C is positief).
     return false; // Homing fase 2 is nog niet voltooid.
   } else { // Homing-schakelaar C is bereikt.
     position1 = HOME_POSITION; // Stel positie teller in op de gedefinieerde thuispositie (0).
     return true; // Homing fase 2 (en dus de hele homing) is voltooid.
   }
 }

 /**
  * @brief Genereert een enkele puls voor de opgegeven servomotor.
  * @param 'servomotor' Het motornummer (1 voor pan, 2 voor tilt).
  * @param delayMicros De duur (in microseconden) voor de HIGH en LOW fase van de puls.
  */
 void stepMotor(int motor, int delayMicros) {
   // Bepaal welke PUL pin gebruikt moet worden.
   int pulPin = (motor == 1) ? PUL1 : PUL2;
   // Genereer de puls.
   digitalWrite(pulPin, HIGH);
   delayMicroseconds(delayMicros); // Duur HIGH.
   digitalWrite(pulPin, LOW);
   delayMicroseconds(delayMicros); // Duur LOW (pauze tussen stappen).
 }

 /**
  * @brief Behandelt de logica voor het afvuren, inclusief modusselectie.
  *        Wordt alleen aangeroepen als het systeem gehomed is.
  */
 void handleFiringLogic() {
   // Bepaal of L2 en R2 beide ingedrukt zijn (gebaseerd op verwerkte waarden uit handleMessage).
   bool l2r2 = (lastL2 == 1 && lastR2 == 1); // lastL2/R2 zijn 0 of 1.
   // 'static' om vorige L1 status en vuurmodus te onthouden voor display update detectie.
   static int prevL1_firing = 0;
   static FiringMode prevFiringMode = SINGLE;

   // --- Vuurmodus Selectie (L1 Knop) ---
   // Detecteer opgaande flank van L1.
   if (prevL1_firing == 0 && l1Value == 1) {
     firingMode = (FiringMode)((firingMode + 1) % 3); // Wissel cyclisch.
     Serial.print("Firing mode: "); Serial.println(firingModeNames[firingMode]);
     // Als de modus daadwerkelijk veranderd is, markeer display voor update.
     if (prevFiringMode != firingMode) {
       displayNeedsUpdate = true; 
       prevFiringMode = firingMode; // Onthoud nieuwe modus.
     }
   }
   // Update vorige L1 status voor volgende iteratie.
   prevL1_firing = l1Value;

   // --- Vuurlogica per Modus ---
   // Tijd van elke vuur modi gebaseerd op 15RPS.

   // --- SINGLE Modus ---
   if (firingMode == SINGLE) {
     // Start puls: opgaande flank L2R2, niet al bezig.
     if (l2r2 && !prevL2R2 && !firingActive) {
       firingActive = true;
       firingStartTime = millis();
       digitalWrite(MAGNET_PIN, HIGH); // Activeer magneet.
     }
     // Stop puls: na 67ms.
     if (firingActive && millis() - firingStartTime >= 67) {
       digitalWrite(MAGNET_PIN, LOW);
       firingActive = false;
     }
     // Stop puls: als L2R2 wordt losgelaten.
     if (!l2r2) {
       digitalWrite(MAGNET_PIN, LOW);
       firingActive = false;
     }
   }
   // --- BURST Modus ---
   else if (firingMode == BURST) {
     // Start puls: opgaande flank L2R2, niet al bezig.
     if (l2r2 && !prevL2R2 && !firingActive) {
       firingActive = true;
       firingStartTime = millis();
       digitalWrite(MAGNET_PIN, HIGH);
     }
     // Stop puls: na 200ms.
     if (firingActive && millis() - firingStartTime >= 200) {
       digitalWrite(MAGNET_PIN, LOW);
       firingActive = false;
     }
     // Stop puls: als L2R2 wordt losgelaten.
     if (!l2r2) {
       digitalWrite(MAGNET_PIN, LOW);
       firingActive = false;
     }
   }
   // --- AUTO Modus ---
   else if (firingMode == AUTO) {
     if (l2r2) { // Als L2R2 ingedrukt zijn.
       digitalWrite(MAGNET_PIN, HIGH); // Houd magneet aan.
     } else { // Als L2R2 losgelaten wordt.
       digitalWrite(MAGNET_PIN, LOW); // Zet magneet uit.
     }
     // firingActive wordt niet gebruikt voor timing in AUTO modus.
     firingActive = false; 
   }
   // Update vorige L2R2 status voor volgende iteratie (flankdetectie).
   prevL2R2 = l2r2;
 } // Einde handleFiringLogic()

 /**
  * @brief Verwerkt een binnenkomend bericht (CSV string) van de ESP32.
  *        Parsert de komma-gescheiden waarden en update de globale statusvariabelen.
  * @param message De volledige regel data ontvangen via Serial2, zonder de newline.
  */
 void handleMessage(const String& message) {
   int field = 0;        // Teller voor het huidige veld (kolom).
   int lastIndex = 0;    // Startpositie van de substring voor het huidige veld.
   // Lokale variabelen om waarden tijdelijk op te slaan tijdens parsen.
   int r1Value_local = 0, l2Value_local = 0, r2Value_local = 0, l1_local_msg = 0, share_local = 0;
   int l3Y_local = 0, r3Y_local = 0;

   // Loop door het bericht karakter voor karakter (inclusief einde string voor laatste veld).
   for (int i = 0; i <= message.length(); i++) { // <= om laatste veld te pakken.
     // Als een komma wordt gevonden OF het einde van het bericht is bereikt:
     if (message[i] == ',' || i == message.length()) {
       // Extraheer de waarde van het huidige veld.
       // message.length() - 1 in de ternary operator was nodig als de ESP32 code een newline in de laatste waarde had.
       // Hier is het veiliger om direct tot 'i' te gaan.
       String valueStr = message.substring(lastIndex, i);
       valueStr.trim(); // Verwijder eventuele witruimte.

       // Wijs de waarde toe aan de juiste lokale variabele op basis van het veldnummer.
       // De veldnummers corresponderen met de volgorde in de ESP32 sprintf.
       if (field == 1) l3Y_local = valueStr.toInt();      // Linker Stick Y
       if (field == 3) r3Y_local = valueStr.toInt();      // Rechter Stick Y
       if (field == 12) l1_local_msg = valueStr.toInt();  // L1 knop
       if (field == 13) r1Value_local = valueStr.toInt(); // R1 knop
       if (field == 14) l2Value_local = valueStr.toInt(); // L2 trigger waarde (ruw 0-255)
       if (field == 15) r2Value_local = valueStr.toInt(); // R2 trigger waarde (ruw 0-255)
       if (field == 16) share_local = valueStr.toInt();   // Share knop
       
       field++; // Ga naar het volgende veld.
       lastIndex = i + 1; // Update de startindex voor het volgende veld (na de komma).
     }
   }

   // Update de globale variabelen met de zojuist geparste waarden.
   l3Y = l3Y_local;
   r3Y = r3Y_local;
   lastR1 = r1Value_local;
   // Belangrijk: De ruwe L2/R2 waarden worden hier opgeslagen.
   // De logica om te bepalen of ze "ingedrukt" zijn (> drempelwaarde)
   // moet elders gebeuren, of hier direct toegepast worden (lastL2 = (l2Value_local > 30) ? 1 : 0;).
   // De huidige vuurlogica (handleFiringLogic) verwacht dat lastL2/R2 0 of 1 zijn.
   // Dus hier een drempelwaarde toepassen is consistent.
   lastL2 = (l2Value_local > 30) ? 1 : 0; // Drempelwaarde 30.
   lastR2 = (r2Value_local > 30) ? 1 : 0; // Drempelwaarde 30.
   l1Value = l1_local_msg;   // Voor moduswissel in loop().
   lastShare = share_local;  // Voor homing in loop().
 } // Einde handleMessage()
