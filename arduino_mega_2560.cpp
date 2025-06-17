

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
 const int LIMIT1_A = 2; // Input pin voor limietschakelaar A voor motor 1.
 const int LIMIT1_B = 3; // Input pin voor limietschakelaar B voor motor 1.
 const int LIMIT3_C = 12; // Input pin voor homing-schakelaar C (middenpositie / referentie) voor motor 1.

 // === Pin Definities Motor 2 (Tilt / Verticale As) ===
 const int PUL2 = 10;  // Pulse (STEP) pin voor servomotor driver 2.
 const int DIR2 = 9;   // Direction (DIR) pin voor servomotor driver 2.
 const int ALM2 = 8;   // Alarm/Enable (ALM/EN) pin voor stappenmotor driver 2.
 const int LIMIT2_A = 4;  // Input pin voor limietschakelaar A voor motor 2.
 const int LIMIT2_B = 11; // Input pin voor limietschakelaar B voor motor 2.

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
 int lastL2 = 0;        // Meest recente status L2 trigger (ontvangen, verwerkt naar 0/1).
 int lastR2 = 0;        // Meest recente status R2 trigger (ontvangen, verwerkt naar 0/1).
 int lastL1 = 0;        // Meest recente status L1 knop (ontvangen).
 int lastShare = 0;     // Meest recente status Share knop (ontvangen, start homing).
 int l1Value = 0;       // Buffer voor L1 waarde uit bericht (voor modus wissel).
 const int DEADZONE = 20; // Dode zone voor de joysticks om kleine afwijkingen te negeren.

 // === Vuurmechanisme Controle Variabelen ===
 bool firingActive = false;             // Geeft aan of het vuurmechanisme momenteel actief is (timing puls).
 unsigned long firingStartTime = 0;    // Tijdstip waarop het vuren begon (voor timing puls).
 bool prevL2R2 = false;                 // Vorige gecombineerde status van L2 en R2 (voor flankdetectie).

 // === Positie Tracking voor servomotoren ===
 long position1 = 0;  // Huidige positie van motor 1 (pan) in stappen, relatief t.o.v. homing positie.
 long position2 = 0;  // Huidige positie van motor 2 (tilt) in stappen.
 const long HOME_POSITION = 0; // Definieert de 'thuis' positie in stappen (na raken LIMIT3_C).

 // === Homing Variabelen & Constanten ===
 bool isHomed = false;         // Vlag die aangeeft of de homing-procedure succesvol is voltooid.
 unsigned long homingTimeout = 30000; // Maximale tijd (in ms) voor de homing-sequentie (30 seconden).
 unsigned long homingStartTime = 0;     // Tijdstip (millis()) waarop de huidige homing fase is gestart.

 // === Homing States (voor de state machine, zorgt voor robuuste werking) ===
 enum HomingState {
   HOMING_IDLE,          // Geen homing actief, of homing voltooid. Systeem wacht op commando.
   HOMING_SEARCH_EDGE,   // Fase 1: Zoek een van de buitenste limieten (A of B) voor de pan-as.
   HOMING_SEARCH_CENTER, // Fase 2: Zoek de middelste (home) limiet (C) voor de pan-as.
   HOMING_COMPLETE       // Homing is succesvol afgerond. Systeem is klaar voor normale operatie.
 };
 HomingState currentHomingState = HOMING_IDLE; // Houdt de huidige staat van de homing-procedure bij.


 // === OLED Display Variabelen ===
 unsigned long lastDisplayUpdate = 0;             // Tijdstip van de laatste display update.
 const unsigned long DISPLAY_UPDATE_INTERVAL = 2000; // Update het display elke 2 seconden (tenzij geforceerd).
 bool displayNeedsUpdate = false;                  // Vlag om een onmiddellijke display update te forceren.
 bool motorActive = false;                       // Vlag om te bepalen of motoren recent bewogen zijn (voor display update timing).

 // Variabelen om de vorige status van limietschakelaars bij te houden voor display updates.
 // Dit helpt om het display alleen te updaten als er daadwerkelijk een verandering is.
 bool prevLimitA1_display = false;
 bool prevLimitB1_display = false;
 bool prevLimitA2_display = false;
 bool prevLimitB2_display = false;
 bool prevLimitC3_display = false;


 // === Functie Prototypes (declaraties) ===
 // Dit stelt de compiler in staat om functies te vinden, zelfs als ze later in de code gedefinieerd zijn.
 void stepMotor(int motor, int delayMicros = 500); // Genereert een stap voor de opgegeven motor.
 void handleMessage(const String& message);       // Verwerkt inkomende berichten van de ESP32.
 void handleFiringLogic();                        // Behandelt de logica voor het afvuren.
 void startHomingSequence();                      // Start de homing-procedure.
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

   // Configureer pinnen voor Motor 1 (pan) als OUTPUT of INPUT_PULLUP.
   pinMode(PUL1, OUTPUT);
   pinMode(DIR1, OUTPUT);
   pinMode(ALM1, OUTPUT);
   pinMode(LIMIT1_A, INPUT_PULLUP); // INPUT_PULLUP: interne weerstand naar VCC, schakelaar trekt naar GND.
   pinMode(LIMIT1_B, INPUT_PULLUP);
   pinMode(LIMIT3_C, INPUT_PULLUP); // Homing-schakelaar.

   // Configureer pinnen voor Motor 2 (tilt) als OUTPUT of INPUT_PULLUP.
   pinMode(PUL2, OUTPUT);
   pinMode(DIR2, OUTPUT);
   pinMode(ALM2, OUTPUT);
   pinMode(LIMIT2_A, INPUT_PULLUP);
   pinMode(LIMIT2_B, INPUT_PULLUP);

   // Configureer overige output pinnen.
   pinMode(R1_PIN, OUTPUT);     // Pin voor laser.
   pinMode(MAGNET_PIN, OUTPUT); // Pin voor elektromagneet.

   // Stel de initiële status van de output pinnen in.
   digitalWrite(ALM1, LOW);     // Activeer motor driver 1.
   digitalWrite(ALM2, LOW);     // Activeer motor driver 2.
   // De initiële richting van DIR1 wordt in startHomingSequence() ingesteld.
   // digitalWrite(DIR1, LOW);
   // digitalWrite(DIR2, HIGH); // Initiële richting voor tilt, kan aangepast worden.
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
       if (inputString.length() < 199) { // Max lengte CSV string is ongeveer 200
          inputString += inChar;
       }
     }
   }

   // Lees de huidige status van alle limietschakelaars één keer aan het begin van de loop
   // om consistentie te garanderen binnen deze loop-iteratie.
   bool limitA1 = (digitalRead(LIMIT1_A) == LOW); // True als schakelaar ingedrukt is.
   bool limitB1 = (digitalRead(LIMIT1_B) == LOW);
   bool limitA2 = (digitalRead(LIMIT2_A) == LOW);
   bool limitB2 = (digitalRead(LIMIT2_B) == LOW);
   bool limitC3 = (digitalRead(LIMIT3_C) == LOW); // Homing-schakelaar (pan midden).

   // Controleer of de status van een limietschakelaar is veranderd sinds de laatste check.
   // Indien ja, zet de vlag om het display te updaten.
   if (limitA1 != prevLimitA1_display || limitB1 != prevLimitB1_display ||
       limitA2 != prevLimitA2_display || limitB2 != prevLimitB2_display ||
       limitC3 != prevLimitC3_display) {
     displayNeedsUpdate = true; // Forceer een display update.
     // Update de 'vorige' statussen voor de volgende vergelijking.
     prevLimitA1_display = limitA1;
     prevLimitB1_display = limitB1;
     prevLimitA2_display = limitA2;
     prevLimitB2_display = limitB2;
     prevLimitC3_display = limitC3;
   }

   // Variabele om de vorige staat van de Share knop bij te houden voor flankdetectie.
   static bool prevShareButtonState = false;
   // Lees de huidige staat van de Share knop (uit de PS4 data via `lastShare`).
   bool currentShareButtonState = (lastShare == 1);

   // === Homing Start Conditie ===
   // Start de homing-sequentie ALLEEN als:
   // 1. De Share-knop NU wordt ingedrukt (was niet, is nu wel).
   // 2. Er GEEN homing-procedure al bezig is (currentHomingState is HOMING_IDLE).
   if (currentShareButtonState && !prevShareButtonState && currentHomingState == HOMING_IDLE) {
     startHomingSequence();            // Roep de functie aan om homing te initialiseren.
     currentHomingState = HOMING_SEARCH_EDGE; // Zet de state machine in de eerste homing fase.
     homingStartTime = millis();       // Start de timer voor een eventuele homing timeout.
     displayNeedsUpdate = true;        // Forceer een display update om de nieuwe homing status te tonen.
   }
   // Update de vorige staat van de Share-knop voor de volgende iteratie.
   prevShareButtonState = currentShareButtonState;

   // === Homing State Machine Logica ===
   // Verwerkt de verschillende fases van de homing-procedure.
   switch (currentHomingState) {
     case HOMING_SEARCH_EDGE: // Fase 1: Zoek een van de buitenste limieten (A of B).
       // De motor beweegt in de richting ingesteld door startHomingSequence().
       if (limitA1) { // Linker limiet (A) is geraakt.
         Serial.println("Phase 1 complete: Reached limit A (pan left).");
         Serial.println("Phase 2: Moving to home position C (pan center)...");
         delay(200); // Korte pauze voor stabiliteit.
         digitalWrite(DIR1, LOW); // Keer de draairichting om: beweeg nu naar rechts (richting C).
         currentHomingState = HOMING_SEARCH_CENTER; // Ga naar de volgende homing fase.
         displayNeedsUpdate = true;
       } else if (limitB1) { // Rechter limiet (B) is geraakt.
         Serial.println("Phase 1 complete: Reached limit B (pan right).");
         Serial.println("Phase 2: Moving to home position C (pan center)...");
         delay(200); // Korte pauze.
         digitalWrite(DIR1, HIGH); // Keer de draairichting om: beweeg nu naar links (richting C).
         currentHomingState = HOMING_SEARCH_CENTER; // Ga naar de volgende homing fase.
         displayNeedsUpdate = true;
       } else { // Geen van de buitenste limieten is nog geraakt.
         stepMotor(1, 400); // Zet een stap met motor 1 (pan). Pulsduur 400us.
         // De richting is al ingesteld in startHomingSequence() of na het raken van de vorige limiet.
       }
       break;

     case HOMING_SEARCH_CENTER: // Fase 2: Zoek de middelste (home) limiet C.
       // De motor beweegt nu in de richting van limiet C.
       if (limitC3) { // Middelste (home) limiet C is geraakt.
         position1 = HOME_POSITION; // Stel de getelde positie van motor 1 in op 0.
         isHomed = true;            // Markeer het systeem als succesvol gehomed.
         currentHomingState = HOMING_COMPLETE; // Homing is voltooid.
         Serial.println("HOMING COMPLETE: Motor positioned at home C (pan center).");
         Serial.println("Normal operation now available.");
         displayNeedsUpdate = true;
       } else { // Limiet C nog niet geraakt.
         stepMotor(1, 400); // Zet een stap met motor 1 (pan).
         // De richting is al ingesteld na het raken van limiet A of B.
       }
       break;

     case HOMING_COMPLETE: // Homing is afgerond.
       currentHomingState = HOMING_IDLE; // Reset de state machine, klaar voor eventuele nieuwe homing.
       displayNeedsUpdate = true;
       break;
       
     case HOMING_IDLE: // Systeem is in rust of wacht op homing commando.
       // Doe hier niets actiefs met betrekking tot homing.
       break;
   }
   
   // === Homing Timeout Check ===
   // Controleer of de homing-procedure te lang duurt (vastgelopen?).
   if (currentHomingState != HOMING_IDLE && currentHomingState != HOMING_COMPLETE && millis() - homingStartTime > homINGTimeout) {
     Serial.println("HOMING TIMEOUT! Check limit switches and motor connections.");
     currentHomingState = HOMING_IDLE; // Stop de homing poging.
     isHomed = false;                   // Systeem is niet (meer) gehomed.
     displayNeedsUpdate = true;
   }

   // === Normale Bediening (alleen als systeem gehomed is en geen homing bezig is) ===
   if (isHomed && currentHomingState == HOMING_IDLE) {
     motorActive = false; // Reset motor activiteit vlag voor deze iteratie.
     
     // --- Motor 1 (Pan) Besturing ---
     // Gebruik 'else if' om te voorkomen dat beide richtingen tegelijk (kortstondig) actief zijn.
     if (l3Y < -DEADZONE && !limitA1) { // Beweeg naar links (negatief Y) indien niet bij limiet A.
       motorActive = true;             // Motor 1 is actief.
       digitalWrite(DIR1, HIGH);       // Stel richting in.
       stepMotor(1);                   // Zet één stap (gebruikt default delayMicros van 500us).
       position1--;                    // Werk positie teller bij.
     } else if (l3Y > DEADZONE && !limitB1) { // Beweeg naar rechts (positief Y) indien niet bij limiet B.
       motorActive = true;             // Motor 1 is actief.
       digitalWrite(DIR1, LOW);        // Stel richting in.
       stepMotor(1);                   // Zet één stap.
       position1++;                    // Werk positie teller bij.
     }

     // --- Motor 2 (Tilt) Besturing ---
     if (r3Y < -DEADZONE && !limitA2) { // Beweeg omhoog (negatief Y) indien niet bij limiet A2 (boven).
       motorActive = true;             // Motor 2 is actief.
       digitalWrite(DIR2, LOW);        // Stel richting in.
       stepMotor(2, 800);              // Zet één stap (langere puls voor tilt?).
       position2--;                    // Werk positie teller bij.
     } else if (r3Y > DEADZONE && !limitB2) { // Beweeg omlaag (positief Y) indien niet bij limiet B2 (onder).
       motorActive = true;             // Motor 2 is actief.
       digitalWrite(DIR2, HIGH);       // Stel richting in.
       stepMotor(2, 800);              // Zet één stap.
       position2++;                    // Werk positie teller bij.
     }

     // --- R1 Knop Logica (Laser Toggle) ---
     // Detecteer neergaande flank: R1 was ingedrukt (1) en is nu losgelaten (0).
     if (lastR1 == 0 && prevR1 == 1) {
       pin23State = (pin23State == LOW) ? HIGH : LOW; // Wissel status.
       digitalWrite(R1_PIN, pin23State);             // Zet pin.
       displayNeedsUpdate = true; // Update display met nieuwe laser status.
     }
   } else if (!isHomed && currentHomingState == HOMING_IDLE) {
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
   // Update het display alleen als het systeem in rust is (geen motor/vuuractie, geen homing)
   // EN als de update interval verstreken is OF een update geforceerd is.
   if (!motorActive && !firingActive && currentHomingState == HOMING_IDLE) {
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
  */
 void updateDisplay() {
   u8g2.clearBuffer(); // Wis buffer.
   u8g2.setFont(u8g2_font_6x10_tf); // Standaard lettertype voor titel.
   
   // Teken titel en scheidingslijn.
   u8g2.drawStr(0, 10, "TURRET DIAGNOSTICS");
   u8g2.drawLine(0, 12, 128, 12); // Horizontale lijn.
   
   // Homing status.
   String homingStatus = "HOME: ";
   if (isHomed) { // Systeem is succesvol gehomed.
       homingStatus += "OK";
   } else { // Systeem is niet gehomed, of homing is bezig.
     switch (currentHomingState) {
       case HOMING_IDLE:          homingStatus += "NOK"; break; // Niet gehomed, wacht op start.
       case HOMING_SEARCH_EDGE:   homingStatus += "->EDGE"; break; // Zoekt buitenste limiet.
       case HOMING_SEARCH_CENTER: homingStatus += "->CENTER"; break; // Zoekt middelste limiet.
       case HOMING_COMPLETE:      homingStatus += "DONE"; break; // Homing net voltooid.
     }
   }
   u8g2.drawStr(0, 24, homingStatus.c_str()); // Teken homing status.
   
   // Vuurmodus.
   String fireStr = "FIRE: " + String(firingModeNames[firingMode]);
   u8g2.drawStr(0, 36, fireStr.c_str()); // Teken vuurmodus.
   
   // Status limietschakelaars (kleiner lettertype voor meer info).
   u8g2.setFont(u8g2_font_5x7_tf); // Kleiner lettertype.
   // Toon labels en status (met '*' indien actief).
   u8g2.drawStr(80, 24, "LIMITS:"); // Sectie titel.
   u8g2.drawStr(80, 33, digitalRead(LIMIT1_A) == LOW ? "A1*" : "A1"); // Pan Links.
   u8g2.drawStr(80, 42, digitalRead(LIMIT1_B) == LOW ? "B1*" : "B1"); // Pan Rechts.
   u8g2.drawStr(80, 51, digitalRead(LIMIT3_C) == LOW ? "C3*" : "C3"); // Pan Midden (Home).
   u8g2.drawStr(100, 33, digitalRead(LIMIT2_A) == LOW ? "A2*" : "A2"); // Tilt Boven/Onder.
   u8g2.drawStr(100, 42, digitalRead(LIMIT2_B) == LOW ? "B2*" : "B2"); // Tilt Onder/Boven.
   
   // Laser status (R1_PIN).
   u8g2.drawStr(80, 60, pin23State == HIGH ? "L:*" : "L:"); // L voor Laser, '*' = aan.
   
   u8g2.sendBuffer(); // Stuur buffer naar display.
 }

 /**
  * @brief Start de homing-sequentie voor motor 1 (pan).
  *        Zet de initiële richting om eerst naar een buitenste limiet te zoeken.
  */
 void startHomingSequence() {
   Serial.println("=== STARTING HOMING SEQUENCE (PAN AXIS) ===");
   Serial.println("Phase 1: Searching for an edge limit (A or B)...");
   isHomed = false; // Markeer systeem als niet-gehomed.
   // Begin met bewegen naar links (richting limiet A).
   digitalWrite(DIR1, HIGH); 
 }

 /**
  * @brief Genereert een enkele puls voor de opgegeven stappenmotor.
  * @param motor Het motornummer (1 voor pan, 2 voor tilt).
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
   // Bepaal of L2 en R2 beide ingedrukt zijn.
   bool l2r2 = (lastL2 == 1 && lastR2 == 1); // Waarden zijn al 0/1 uit handleMessage.
   // 'static' om vorige L1 status en vuurmodus te onthouden.
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
   // De veiligheidscheck op de hoek is in deze versie niet aanwezig in de vuurlogica zelf.
   // Dit zou eventueel vóór het aanroepen van handleFiringLogic() gecontroleerd kunnen worden.

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
     if (!l2r2) { // Directe check op huidige L2R2 status.
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
   // De ruwe L2/R2 waarden (0-255) worden hier ontvangen.
   // De handleFiringLogic functie verwacht dat lastL2/lastR2 0 of 1 zijn.
   // Daarom wordt hier een drempelwaarde toegepast.
   lastL2 = (l2Value_local > 30) ? 1 : 0; // Als L2 waarde > 30, dan 1 (ingedrukt), anders 0.
   lastR2 = (r2Value_local > 30) ? 1 : 0; // Als R2 waarde > 30, dan 1 (ingedrukt), anders 0.
   l1Value = l1_local_msg;   // Voor moduswissel.
   lastShare = share_local;  // Voor starten homing.
 } // Einde handleMessage()
