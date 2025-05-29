 /************************************************************************************
 *	██████╗ ███████╗██╗  ██╗              ███████╗███████╗██████╗ ██████╗ ██████╗ 
 *	██╔══██╗██╔════╝██║  ██║              ██╔════╝██╔════╝██╔══██╗╚════██╗╚════██╗
 *	██████╔╝███████╗███████║    █████╗    █████╗  ███████╗██████╔╝ █████╔╝ █████╔╝
 *	██╔═══╝ ╚════██║╚════██║    ╚════╝    ██╔══╝  ╚════██║██╔═══╝  ╚═══██╗██╔═══╝ 
 *	██║     ███████║     ██║              ███████╗███████║██║     ██████╔╝███████╗
 *	╚═╝     ╚══════╝     ╚═╝              ╚══════╝╚══════╝╚═╝     ╚═════╝ ╚══════╝                                                                                        
 *
 *         ESP32 - PLAYSTATION 4 CONTROLLER INTERFACE FOR RC AIRSOFT TURRET
 *
 *************************************************************************************
 * Project:      Graduaatsproef Afstandbestuurd Wapen Systeem (Airsoft Demonstrator)
 * Context:      Marinebasis Zeebrugge - Demonstratiemodel voor evenementen/beurzen
 * Module:       ESP32 Controller Interface
 * Author:       D'Haenens Jorrit
 * Date:         04-29-2025
 * Version:      1.0
 *
 * Platform:     ESP32 - NodeMCU (ESP32-WROOM-32)
 * Purpose:      Deze code draait op de ESP32 en is verantwoordelijk voor:
 *               - Het verbinden met een Playstation 4 (PS4) controller via Bluetooth.
 *               - Het continu uitlezen van de status van knoppen, joysticks en triggers.
 *               - Het formatteren van de controller data in een Comma Separated Value (CSV) string.
 *               - Het periodiek versturen van deze CSV data via:
 *                  - De standaard Serial poort (USB) voor debugging.
 *                  - De Serial2 poort voor communicatie met de Arduino Mega (turret controller).
 *               - Het afhandelen van connectie- en disconnectie-events van de controller.
 *
 * Libraries:    - PS4Controller.h (by Yoshino Taro - https://github.com/aed3/PS4-esp32)
 *
 * Communication: - Bluetooth (naar PS4 Controller)
 *                - Serial (USB - Debug Output)
 *                - Serial2 (GPIO 16/17 - Naar Arduino Mega)
 *
 * !! DISCLAIMER !!
 * Dit systeem is ontwikkeld als een NIET-LETHAAL demonstratiemodel ("speelgoed")
 * met een airsoft-apparaat. Het is uitsluitend bedoeld voor educatieve en
 * representatieve doeleinden tijdens gecontroleerde evenementen en beurzen.
 * Dit is GEEN operationeel wapensysteem. Behandel het airsoft-apparaat altijd
 * met de nodige veiligheidsmaatregelen.
 ************************************************************************************/
 
 #include <PS4Controller.h> // Importeer de bibliotheek voor PS4 Controller functionaliteit via Bluetooth.
 #include <Wire.h>          // Importeer de Wire bibliotheek voor I2C communicatie (vereist door U8g2).
 #include <U8g2lib.h>       // Importeer de bibliotheek voor het aansturen van monochrome displays.

 // === Pin Definities voor I2C (OLED Display) ===
 // Definieer de pinnen die gebruikt worden voor de Software I2C communicatie met het OLED display.
 // Dit is nodig omdat Software I2C gebruikt wordt (zie U8g2 constructor).
 #define I2C_SDA 21 // GPIO pin gebruikt voor SDA (Data).
 #define I2C_SCL 18 // GPIO pin gebruikt voor SCL (Clock). Let op: Pin 22 is vaak standaard, hier wordt 18 gebruikt!

 // Initialiseer het U8g2 display object voor een SSD1306 128x64 OLED display.
 // U8G2_R0: Geen rotatie.
 // _F_: Full buffer mode (gebruikt meer RAM, maar is flexibeler).
 // _SW_I2C: Gebruik Software I2C implementatie (bit-banging).
 // I2C_SCL, I2C_SDA: De GPIO pinnen gedefinieerd hierboven.
 // U8X8_PIN_NONE: Geen aparte reset pin gebruikt.
 U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
   U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);

 // === Globale Variabelen ===
 unsigned long lastTimeStamp = 0; // Variabele om de tijd bij te houden sinds de laatste keer data is verzonden via Serial (voor rate limiting).

 // === Functies voor OLED Display ===

 /**
  * @brief Werkt het OLED display bij met een statisch bericht.
  *        Tekent een kader, een titelbalk en centreert het opgegeven bericht.
  * @param message De tekst die in het midden van het scherm getoond moet worden.
  */
 void updateOLED(const char* message) {
   u8g2.clearBuffer();                // Wis de interne display buffer.
   u8g2.setFont(u8g2_font_6x10_tf); // Kies een compact lettertype (6 pixels breed, 10 hoog).
   // Bereken de breedte van het bericht om het te kunnen centreren.
   int16_t text_width = u8g2.getStrWidth(message);
   // Bereken de X-positie voor horizontale centrering (128 = schermbreedte).
   int16_t x = (128 - text_width) / 2;
   u8g2.drawFrame(0, 0, 128, 64);         // Teken een kader rond het hele scherm.
   u8g2.drawHLine(0, 12, 128);            // Teken een horizontale lijn als titelbalk scheiding.
   u8g2.setCursor(4, 10);                 // Zet de cursor voor de titel (net boven de lijn).
   u8g2.print("== SYSTEM STATUS ==");      // Print de titel.
   u8g2.setCursor(x, 38);                 // Zet de cursor op de berekende X-positie, verticaal gecentreerd (rond y=38).
   u8g2.print(message);                   // Print het opgegeven bericht.
   u8g2.sendBuffer();                   // Stuur de inhoud van de buffer naar het fysieke display.
 }

 /**
  * @brief Werkt het OLED display bij met een scrollend bericht.
  *        Behoudt het kader en de titel, maar laat het opgegeven bericht van rechts naar links scrollen.
  * @param message De tekst die moet scrollen.
  */
 void scrollText(const char* message) {
   // Statische variabelen behouden hun waarde tussen functieaanroepen.
   static int16_t offset = 128;        // Huidige X-offset voor de tekst, start rechts buiten beeld.
   static unsigned long lastScroll = 0; // Tijdstip van de laatste scroll-stap.

   // Controleer of het tijd is voor de volgende scroll-stap (beperkt de snelheid).
   if (millis() - lastScroll > 30) { // 30ms interval bepaalt de scrollsnelheid.
     lastScroll = millis();             // Update de tijd van de laatste stap.
     u8g2.clearBuffer();                // Wis de buffer.
     u8g2.setFont(u8g2_font_6x10_tf);   // Stel het lettertype in.
     u8g2.drawFrame(0, 0, 128, 64);         // Teken de randen.
     u8g2.drawHLine(0, 12, 128);            // Teken de bovenlijn.
     u8g2.setCursor(4, 10);                 // Zet cursor voor de titel.
     u8g2.print("== SYSTEM STATUS ==");     // Print de titel.

     u8g2.setCursor(offset, 38);            // Zet de cursor op de huidige scroll-offset.
     u8g2.print(message);                   // Print de scrollende tekst.
     u8g2.sendBuffer();                   // Stuur naar het display.

     offset -= 2; // Verschuif de offset 2 pixels naar links voor de volgende stap.

     // Als de tekst volledig links uit beeld is verdwenen:
     // Bereken de breedte van de tekst. Als de offset negatiever is dan de negatieve breedte, reset.
     if (offset < -u8g2.getStrWidth(message)) {
       offset = 128; // Reset de offset naar de startpositie (rechts buiten beeld).
     }
   }
 }

 // === PS4 Controller Callback Functies ===

 /**
  * @brief Callback functie die wordt aangeroepen door de PS4Controller bibliotheek
  *        telkens wanneer er nieuwe data van de controller beschikbaar is.
  *        Formatteert de data en stuurt deze via Serial en Serial2.
  */
 void notify() {
   // Buffer om de geformatteerde data string in op te slaan.
   // Zorg ervoor dat deze groot genoeg is voor alle data + komma's + null terminator.
   char messageString[200];

   // Format de controller data in een comma-separated string (CSV).
   // Elk %-teken geeft aan waar een waarde moet worden ingevoegd.
   // %4d: een integer (getal), uitgelijnd in een veld van 4 karakters.
   // %3d: een integer (getal), uitgelijnd in een veld van 3 karakters.
   sprintf(messageString,
     "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d," // Sticks, D-Pad, Knoppen
     "%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d", // L1/R1, L2/R2 (waarden), Share/Options/PS/Touchpad, Status
     PS4.LStickX(), PS4.LStickY(), PS4.RStickX(), PS4.RStickY(), // Joysticks (-128 tot 127)
     PS4.Left(), PS4.Down(), PS4.Right(), PS4.Up(),             // D-Pad (0 of 1)
     PS4.Square(), PS4.Cross(), PS4.Circle(), PS4.Triangle(),    // Actieknoppen (0 of 1)
     PS4.L1(), PS4.R1(),                                        // Schouderknoppen (0 of 1)
     PS4.L2(), PS4.R2(),                                        // Triggers (waarde 0-255)
     PS4.Share(), PS4.Options(), PS4.PSButton(), PS4.Touchpad(), // Speciale knoppen (0 of 1)
     PS4.Charging(), PS4.Audio(), PS4.Mic(), PS4.Battery()       // Status info (0/1, 0/1, 0/1, 0-100 ca.)
   );

   // Rate limiting: Stuur data maximaal elke 50 milliseconden om de seriële poorten niet te overspoelen.
   if (millis() - lastTimeStamp > 50) {
     // Stuur de data naar de standaard Serial poort (USB) voor debugging.
     Serial.println(messageString);
     // Stuur de data naar de Serial2 poort (verbonden met Arduino Mega).
     Serial2.println(messageString);
     // Update de timestamp van de laatste verzending.
     lastTimeStamp = millis();
   }
 }

 /**
  * @brief Callback functie die wordt aangeroepen wanneer de PS4 controller succesvol verbinding maakt.
  */
 void onConnect() {
   // Meld verbinding op de debug Serial poort.
   Serial.println("PS4 Controller Connected!");
   // Meld verbinding aan de Arduino Mega via Serial2.
   Serial2.println("STATUS:PS4_CONNECTED"); // Duidelijke statusmelding voor de Mega.
   // Update het OLED display met een succesbericht.
   updateOLED("[OK] Weapon Locked On");
 }

 /**
  * @brief Callback functie die wordt aangeroepen wanneer de PS4 controller de verbinding verbreekt.
  */
 void onDisConnect() {
   // Meld verbroken verbinding op de debug Serial poort.
   Serial.println("PS4 Controller Disconnected!");
   // Meld verbroken verbinding aan de Arduino Mega via Serial2.
   Serial2.println("STATUS:PS4_DISCONNECTED"); // Duidelijke statusmelding voor de Mega.
   // Update het OLED display met een foutbericht.
   updateOLED("[FAIL] Weapon Offline");
 }

 // === Hoofd Programma ===

 /**
  * @brief Setup functie: Wordt eenmalig uitgevoerd bij het opstarten van de ESP32.
  */
 void setup() {
   // Initialiseer de standaard Serial poort (USB) voor debug output op 115200 baud.
   Serial.begin(115200);
   // Initialiseer de tweede hardware Serial poort (Serial2, standaard GPIO 16/17)
   // voor communicatie met de Arduino Mega op 115200 baud.
   Serial2.begin(115200);
   // Initialiseer het OLED display via de U8g2 bibliotheek.
   u8g2.begin();

   // Toon een initialisatiebericht op het OLED display.
   updateOLED(">> Initializing...");

   // Registreer de callback functies bij de PS4Controller bibliotheek.
   PS4.attach(notify);           // Koppel 'notify' aan data-updates.
   PS4.attachOnConnect(onConnect); // Koppel 'onConnect' aan verbindingsgebeurtenis.
   PS4.attachOnDisconnect(onDisConnect); // Koppel 'onDisConnect' aan verbrekingsgebeurtenis.

   // Start de PS4Controller bibliotheek. Dit initieert Bluetooth en begint te luisteren.
   // Zonder MAC-adres probeert het te verbinden met de laatst bekende controller.
   // Voor de eerste keer koppelen: houd PS + Share ingedrukt op de controller.
   PS4.begin();
   // Optioneel: PS4.begin("DE:AD:BE:EF:FE:ED"); // Verbind met specifiek MAC adres.

   // Meld gereedheid op de debug Serial poort.
   Serial.println("ESP32 Ready. Waiting for PS4 Controller connection...");
   // Meld gereedheid aan de Arduino Mega.
   Serial2.println("STATUS:ESP32_READY");

   // Wacht even zodat het init bericht zichtbaar is.
   delay(1000);
   // Update OLED met een wachtbericht.
   updateOLED("[READY] Awaiting Connection");
 }

 /**
  * @brief Hoofd loop functie: Wordt continu herhaald na setup().
  */
 void loop() {
   // De PS4Controller bibliotheek handelt de Bluetooth communicatie en het aanroepen
   // van de callbacks ('notify', 'onConnect', 'onDisConnect') op de achtergrond af.
   // Daarom hoeft er in de hoofdloop niet veel te gebeuren voor de PS4 functionaliteit.

   // Roep de functie aan om de statusboodschap te laten scrollen op het OLED display.
   // Dit zorgt voor een continue activiteit op het display.
   scrollText("[READY] Weapon Online - Awaiting Command - Secure Link Active");

   // Een kleine delay kan soms helpen bij stabiliteit op sommige ESP32 varianten,
   // maar is meestal niet strikt noodzakelijk als de callbacks goed werken.
   // delay(1);
 }
