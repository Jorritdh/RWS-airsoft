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

// === Globale Variabelen ===
unsigned long lastTimeStamp = 0; // Variabele om de tijd bij te houden sinds de laatste keer data is verzonden via Serial (voor rate limiting).

// === PS4 Controller Callback Functies ===

/**
 * @brief Callback functie die wordt aangeroepen door de PS4Controller bibliotheek
 *        telkens wanneer er nieuwe data van de controller beschikbaar is.
 *        Formatteert de data en stuurt deze via Serial en Serial2.
 */
void notify() {
   // Buffer om de geformatteerde data string in op te slaan.
   char messageString[200];

   // Format de controller data in een comma-separated string (CSV).
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

   // Rate limiting: Stuur data maximaal elke 50 milliseconden.
   if (millis() - lastTimeStamp > 50) {
     Serial.println(messageString); // Stuur naar USB Serial (debug).
     Serial2.println(messageString); // Stuur naar Arduino Mega.
     lastTimeStamp = millis(); // Update timestamp.
   }
}

/**
 * @brief Callback functie die wordt aangeroepen wanneer de PS4 controller succesvol verbinding maakt.
 */
void onConnect() {
   Serial.println("PS4 Controller Connected!");
   Serial2.println("STATUS:PS4_CONNECTED");
}

/**
 * @brief Callback functie die wordt aangeroepen wanneer de PS4 controller de verbinding verbreekt.
 */
void onDisConnect() {
   Serial.println("PS4 Controller Disconnected!");
   Serial2.println("STATUS:PS4_DISCONNECTED");
}

// === Hoofd Programma ===

/**
 * @brief Setup functie: Wordt eenmalig uitgevoerd bij het opstarten van de ESP32.
 */
void setup() {
   Serial.begin(115200); // Initialiseer USB Serial voor debug.
   Serial2.begin(115200); // Initialiseer Serial2 voor Arduino Mega.

   // Registreer de callback functies bij de PS4Controller bibliotheek.
   PS4.attach(notify);
   PS4.attachOnConnect(onConnect);
   PS4.attachOnDisconnect(onDisConnect);

   // Start de PS4Controller bibliotheek (Bluetooth).
   PS4.begin();

   // Meld gereedheid.
   Serial.println("ESP32 Ready. Waiting for PS4 Controller connection...");
   Serial2.println("STATUS:ESP32_READY");
}

/**
 * @brief Hoofd loop functie: Wordt continu herhaald na setup().
 */
void loop() {
   // PS4Controller bibliotheek handelt callbacks af op de achtergrond.
   // Geen extra code nodig in de loop.
}
