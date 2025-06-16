homing fucntie wacht momenteel op 1 limit switch

#include <Wire.h>
#include <U8g2lib.h>

// OLED Display setup (SSH1106, I2C op pins 20 & 21)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Motor 1 (pan)
const int PUL1 = 7;
const int DIR1 = 6;
const int ALM1 = 5;
const int LIMIT1_A = 2; // Eindschakelaar A voor motor 1 (links)
const int LIMIT1_B = 3; // Eindschakelaar B voor motor 1 (rechts)
const int LIMIT3_C = 12; // Homing schakelaar C (midden)

// Motor 2 (tilt)
const int PUL2 = 10;
const int DIR2 = 9;
const int ALM2 = 8;
const int LIMIT2_A = 4;  // Eindschakelaar A voor motor 2
const int LIMIT2_B = 11; // Eindschakelaar B voor motor 2

#define R1_PIN 23
#define MAGNET_PIN 25

// Firing modes
enum FiringMode { SINGLE, BURST, AUTO };
FiringMode firingMode = SINGLE;
const char* firingModeNames[] = {"Single", "Burst", "Auto"};

// Controlevariabelen
int l3Y = 0, r3Y = 0;
int prevR1 = 0;
int pin23State = LOW;
int lastR1 = 0, lastL2 = 0, lastR2 = 0, lastL1 = 0;
int lastShare = 0;
int l1Value = 0;
const int DEADZONE = 20;

// Firing mechanisme
bool firingActive = false;
unsigned long firingStartTime = 0;
bool prevL2R2 = false;

// Positie-tracking
long position1 = 0;
long position2 = 0;
const long HOME_POSITION = 0;

// Homing variabelen
bool isHomed = false;
unsigned long homingTimeout = 30000;
unsigned long homingStartTime = 0;

// Homing states voor robuuste werking
enum HomingState {
  HOMING_IDLE,
  HOMING_SEARCH_EDGE,   // Fase 1: Zoek een van de buitenste limieten
  HOMING_SEARCH_CENTER, // Fase 2: Zoek de middelste limiet
  HOMING_COMPLETE
};
HomingState currentHomingState = HOMING_IDLE;


// OLED Display variabelen
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 2000;
bool displayNeedsUpdate = false;
bool motorActive = false;

bool prevLimitA1_display = false;
bool prevLimitB1_display = false;
bool prevLimitA2_display = false;
bool prevLimitB2_display = false;
bool prevLimitC3_display = false;


// Functie Prototypes
void stepMotor(int motor, int delayMicros = 500);
void handleMessage(const String& message);
void handleFiringLogic();
void startHomingSequence();
void updateDisplay();
void initDisplay();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  initDisplay();

  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(ALM1, OUTPUT);
  pinMode(LIMIT1_A, INPUT_PULLUP);
  pinMode(LIMIT1_B, INPUT_PULLUP);
  pinMode(LIMIT3_C, INPUT_PULLUP);

  pinMode(PUL2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ALM2, OUTPUT);
  pinMode(LIMIT2_A, INPUT_PULLUP);
  pinMode(LIMIT2_B, INPUT_PULLUP);

  pinMode(R1_PIN, OUTPUT);
  pinMode(MAGNET_PIN, OUTPUT);

  digitalWrite(ALM1, LOW);
  digitalWrite(ALM2, LOW);
  digitalWrite(R1_PIN, pin23State);
  digitalWrite(MAGNET_PIN, LOW);

  Serial.println("System ready. Press SHARE to start homing sequence.");
  Serial.println("IMPORTANT: Homing must be completed before normal operation!");
  
  // Initialiseer display-statussen
  prevLimitA1_display = (digitalRead(LIMIT1_A) == LOW);
  prevLimitB1_display = (digitalRead(LIMIT1_B) == LOW);
  prevLimitA2_display = (digitalRead(LIMIT2_A) == LOW);
  prevLimitB2_display = (digitalRead(LIMIT2_B) == LOW);
  prevLimitC3_display = (digitalRead(LIMIT3_C) == LOW);

  updateDisplay();
}

void loop() {
  static String inputString = "";
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (inChar == '\n') {
      handleMessage(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  // Lees alle limietschakelaars één keer aan het begin van de loop
  bool limitA1 = (digitalRead(LIMIT1_A) == LOW);
  bool limitB1 = (digitalRead(LIMIT1_B) == LOW);
  bool limitA2 = (digitalRead(LIMIT2_A) == LOW);
  bool limitB2 = (digitalRead(LIMIT2_B) == LOW);
  bool limitC3 = (digitalRead(LIMIT3_C) == LOW);

  // Check of display een update nodig heeft door veranderde limiet-status
  if (limitA1 != prevLimitA1_display || limitB1 != prevLimitB1_display ||
      limitA2 != prevLimitA2_display || limitB2 != prevLimitB2_display ||
      limitC3 != prevLimitC3_display) {
    displayNeedsUpdate = true;
    prevLimitA1_display = limitA1;
    prevLimitB1_display = limitB1;
    prevLimitA2_display = limitA2;
    prevLimitB2_display = limitB2;
    prevLimitC3_display = limitC3;
  }

  static bool prevShareButtonState = false;
  bool currentShareButtonState = (lastShare == 1);

  // --- DIT IS HET BELANGRIJKSTE GEDEELTE VOOR UW VRAAG ---
  // De homing-sequentie wordt alleen gestart als de Share-knop wordt INGEDRUKT
  // (vorige staat was 'niet ingedrukt', huidige is 'wel ingedrukt')
  // EN als het systeem niet al aan het homen is (state is HOMING_IDLE).
  if (currentShareButtonState && !prevShareButtonState && currentHomingState == HOMING_IDLE) {
    startHomingSequence();
    currentHomingState = HOMING_SEARCH_EDGE;
    homingStartTime = millis();
    displayNeedsUpdate = true; 
  }
  prevShareButtonState = currentShareButtonState;

  // --- Homing State Machine ---
  switch (currentHomingState) {
    case HOMING_SEARCH_EDGE:
      if (limitA1) { // Linker limiet (A) gevonden
        Serial.println("Phase 1 complete: Reached limit A.");
        Serial.println("Phase 2: Moving to home position C...");
        delay(200);
        digitalWrite(DIR1, LOW); // Omkeren, beweeg naar rechts richting C
        currentHomingState = HOMING_SEARCH_CENTER;
        displayNeedsUpdate = true;
      } else if (limitB1) { // Rechter limiet (B) gevonden
        Serial.println("Phase 1 complete: Reached limit B.");
        Serial.println("Phase 2: Moving to home position C...");
        delay(200);
        digitalWrite(DIR1, HIGH); // Omkeren, beweeg naar links richting C
        currentHomingState = HOMING_SEARCH_CENTER;
        displayNeedsUpdate = true;
      } else { // Geen limiet geraakt, blijf bewegen
        stepMotor(1, 400); 
      }
      break;

    case HOMING_SEARCH_CENTER:
      if (limitC3) { // Home positie (C) gevonden
        position1 = HOME_POSITION;
        isHomed = true;
        currentHomingState = HOMING_COMPLETE;
        Serial.println("HOMING COMPLETE: Motor positioned at home C.");
        Serial.println("Normal operation now available.");
        displayNeedsUpdate = true;
      } else { // Nog niet bij C, blijf bewegen
        stepMotor(1, 400);
      }
      break;

    case HOMING_COMPLETE:
      currentHomingState = HOMING_IDLE; // Reset state machine voor volgende keer
      displayNeedsUpdate = true;
      break;
      
    case HOMING_IDLE:
      // Doe niets, wacht op actie van de gebruiker (zoals de Share knop).
      break;
  }
  
  // Homing timeout check
  if (currentHomingState != HOMING_IDLE && currentHomingState != HOMING_COMPLETE && millis() - homingStartTime > homingTimeout) {
    Serial.println("HOMING TIMEOUT! Check switches and connections.");
    currentHomingState = HOMING_IDLE;
    displayNeedsUpdate = true;
  }

  // Normale operatie alleen als homing is voltooid
  if (isHomed && currentHomingState == HOMING_IDLE) {
    motorActive = false;
    
    // Motor 1 (pan) - met 'else if' voor betere structuur
    if (l3Y < -DEADZONE && !limitB1) {
      motorActive = true;
      digitalWrite(DIR1, HIGH);
      stepMotor(1);
      position1--;
    } else if (l3Y > DEADZONE && !limitA1) {
      motorActive = true;
      digitalWrite(DIR1, LOW);
      stepMotor(1);
      position1++;
    }

    // Motor 2 (tilt)
    if (r3Y < -DEADZONE && !limitA2) {
      motorActive = true;
      digitalWrite(DIR2, LOW); 
      stepMotor(2, 800);
      position2--;
    } else if (r3Y > DEADZONE && !limitB2) {
      motorActive = true;
      digitalWrite(DIR2, HIGH);
      stepMotor(2, 800);
      position2++;
    }

    // R1 knop logica
    if (lastR1 == 0 && prevR1 == 1) {
      pin23State = (pin23State == LOW) ? HIGH : LOW;
      digitalWrite(R1_PIN, pin23State);
      displayNeedsUpdate = true;
    }
  } else if (!isHomed && currentHomingState == HOMING_IDLE) {
    // Geef een herinnering als het systeem niet gehomed is.
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 5000) {
      Serial.println("WARNING: System not homed! Press SHARE to start homing.");
      lastWarning = millis();
    }
  }

  prevR1 = lastR1;

  if (isHomed) {
    handleFiringLogic();
  }

  // Update het display alleen als het systeem in rust is
  if (!motorActive && !firingActive && currentHomingState == HOMING_IDLE) {
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL || displayNeedsUpdate) {
      updateDisplay();
      lastDisplayUpdate = millis();
      displayNeedsUpdate = false;
    }
  }
}

void initDisplay() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "Turret System");
  u8g2.drawStr(0, 30, "Initializing...");
  u8g2.sendBuffer();
  delay(1000);
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  
  u8g2.drawStr(0, 10, "TURRET DIAGNOSTICS");
  u8g2.drawLine(0, 12, 128, 12);
  
  String homingStatus = "HOME: ";
  if (isHomed) {
      homingStatus += "OK";
  } else {
    switch (currentHomingState) {
      case HOMING_IDLE:          homingStatus += "NOK"; break;
      case HOMING_SEARCH_EDGE:   homingStatus += "->EDGE"; break;
      case HOMING_SEARCH_CENTER: homingStatus += "->CENTER"; break;
      case HOMING_COMPLETE:      homingStatus += "DONE"; break; 
    }
  }
  u8g2.drawStr(0, 24, homingStatus.c_str());
  
  String fireStr = "FIRE: " + String(firingModeNames[firingMode]);
  u8g2.drawStr(0, 36, fireStr.c_str());
  
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(80, 24, "LIMITS:");
  u8g2.drawStr(80, 33, digitalRead(LIMIT1_A) == LOW ? "A1*" : "A1");
  u8g2.drawStr(80, 42, digitalRead(LIMIT1_B) == LOW ? "B1*" : "B1");
  u8g2.drawStr(80, 51, digitalRead(LIMIT3_C) == LOW ? "C3*" : "C3");
  u8g2.drawStr(100, 33, digitalRead(LIMIT2_A) == LOW ? "A2*" : "A2");
  u8g2.drawStr(100, 42, digitalRead(LIMIT2_B) == LOW ? "B2*" : "B2");
  
  u8g2.drawStr(80, 60, pin23State == HIGH ? "L:*" : "L:");
  
  u8g2.sendBuffer();
}

void startHomingSequence() {
  Serial.println("=== STARTING HOMING SEQUENCE ===");
  Serial.println("Phase 1: Searching for an edge limit (A or B)...");
  isHomed = false;
  // Begin met bewegen naar links (richting limiet A). DIR HIGH is links.
  digitalWrite(DIR1, HIGH); 
}

void stepMotor(int motor, int delayMicros) {
  int pulPin = (motor == 1) ? PUL1 : PUL2;
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(delayMicros);
  digitalWrite(pulPin, LOW);
  delayMicroseconds(delayMicros);
}

void handleFiringLogic() {
  bool l2r2 = (lastL2 == 1 && lastR2 == 1);
  static int prevL1_firing = 0;
  static FiringMode prevFiringMode = SINGLE;

  if (prevL1_firing == 0 && l1Value == 1) {
    firingMode = (FiringMode)((firingMode + 1) % 3);
    Serial.print("Firing mode: "); Serial.println(firingModeNames[firingMode]);
    if (prevFiringMode != firingMode) {
      displayNeedsUpdate = true; 
      prevFiringMode = firingMode;
    }
  }
  prevL1_firing = l1Value;

  if (firingMode == SINGLE) {
    if (l2r2 && !prevL2R2 && !firingActive) {
      firingActive = true;
      firingStartTime = millis();
      digitalWrite(MAGNET_PIN, HIGH);
    }
    if (firingActive && millis() - firingStartTime >= 67) {
      digitalWrite(MAGNET_PIN, LOW);
      firingActive = false;
    }
    if (!l2r2) {
      digitalWrite(MAGNET_PIN, LOW);
      firingActive = false;
    }
  } else if (firingMode == BURST) {
    if (l2r2 && !prevL2R2 && !firingActive) {
      firingActive = true;
      firingStartTime = millis();
      digitalWrite(MAGNET_PIN, HIGH);
    }
    if (firingActive && millis() - firingStartTime >= 200) {
      digitalWrite(MAGNET_PIN, LOW);
      firingActive = false;
    }
    if (!l2r2) {
      digitalWrite(MAGNET_PIN, LOW);
      firingActive = false;
    }
  } else if (firingMode == AUTO) {
    if (l2r2) {
      digitalWrite(MAGNET_PIN, HIGH);
    } else {
      digitalWrite(MAGNET_PIN, LOW);
    }
    firingActive = false; 
  }
  prevL2R2 = l2r2;
}

void handleMessage(const String& message) {
  int field = 0;
  int lastIndex = 0;
  // Lokale variabelen om de data uit de seriële boodschap tijdelijk op te slaan
  int r1Value_local = 0, l2Value_local = 0, r2Value_local = 0, l1_local_msg = 0, share_local = 0;
  int l3Y_local = 0, r3Y_local = 0;

  for (int i = 0; i < message.length(); i++) {
    if (message[i] == ',' || i == message.length() - 1) {
      String valueStr = message.substring(lastIndex, i + (i == message.length() - 1 ? 1 : 0));
      if (field == 1) l3Y_local = valueStr.toInt();
      if (field == 3) r3Y_local = valueStr.toInt();
      if (field == 12) l1_local_msg = valueStr.toInt();
      if (field == 13) r1Value_local = valueStr.toInt();
      if (field == 14) l2Value_local = valueStr.toInt();
      if (field == 15) r2Value_local = valueStr.toInt();
      if (field == 16) share_local = valueStr.toInt();
      field++;
      lastIndex = i + 1;
    }
  }

  // Wijs de gelezen waarden toe aan de globale variabelen
  l3Y = l3Y_local;
  r3Y = r3Y_local;
  lastR1 = r1Value_local;
  lastL2 = l2Value_local;
  lastR2 = r2Value_local;
  l1Value = l1_local_msg;
  lastShare = share_local;
}
