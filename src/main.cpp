#include <Arduino.h>
#include <math.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <Wire.h>

// --- INITIALISATION BLUETOOTH --
BluetoothSerial SerialBT;

// --- PINS DÉFINITION ---
// User LED
#define LEDU1 25
#define LEDU2 26

// Moteurs
#define EN_D 23
#define EN_G 4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

// Encodeurs
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

#define MAX_POINTS 100

// --- EEPROM Configuration ---
#define EEPROM_SIZE 64
#define AUTO_CALIB_FLAG 0xCC

// --- I2C Addresses ---
#define ADDR_MAG 0x1E

// —– Définition de Point (avant tout tableau qui l'utilise) —–
struct Point {
  float x;
  float y;
};

Point PARCOURS[MAX_POINTS];
int NB_POINTS = 0;


// --- PARAMÈTRES ROBOT ---
const float RAYON_ROUE = 0.045;         // 4.5cm
const float ENTRAXE = 0.075;            // 7.5cm
const int TICKS_PAR_TOUR = 1095;
const float PERIMETRE = 2 * PI * RAYON_ROUE;
const float OFFSET_STYLO = 0.13;        // 13cm devant le centre

bool parcoursCharge = false;  // false tant que tu n'as pas envoyé de commande

// --- DÉFINITION DU PARCOURS ---
// Modifiez ici les coordonnées des points cibles (en mètres)

//Je pense que si y a qqch à améliorer c'est ici, faut voir mais je pense qu'on peut trouver des paramètres pour faire parfaitement l'angle droit
const Point baseparcours[] = {
  {0.20, 0.00},      // Avancer de 20cm
  {0.20, 0.11},      // Angle droit 10cm
  {0.60, 0.11}      // Avancer 40cm

      // Retour final
};

const int NB_BASE = sizeof(baseparcours) / sizeof(Point);
int pointActuel = 0;
bool parcoursFini = false;
//Je pense que si y a qqch à améliorer c'est ici, faut voir mais je pense qu'on peut trouver des paramètres pour faire parfaitement l'angle droit
// --- PARAMÈTRES CONTRÔLE ---
const float VITESSE_MAX = 200;          // PWM max
const float VITESSE_MIN = 60;           // PWM min pour vaincre l'inertie
const float DISTANCE_TOLERANCE = 0.02;  // 2cm de précision
const float ANGLE_TOLERANCE = 0.05;     // ~3 degrés
//Je pense que si y a qqch à améliorer c'est ici, faut voir mais je pense qu'on peut trouver des paramètres pour faire parfaitement l'angle droit
// --- PARAMÈTRES PID ---
// PID Distance
const float KP_DIST = 900.0;
const float KI_DIST = 50.0;
const float KD_DIST = 150.0;

// PID Angle
const float KP_ANGLE = 145.0;
const float KI_ANGLE = 15.0;
const float KD_ANGLE = 40.0;

// --- VARIABLES ODOMÉTRIE ---
volatile long ticksG = 0, ticksD = 0;
long lastTicksG = 0, lastTicksD = 0;

// Position du robot (centre)
double x_robot = -OFFSET_STYLO;
double y_robot = 0.0;
double theta_robot = 0.0;

// --- VARIABLES PID ---
float erreurDist_precedente = 0;
float integrale_dist = 0;
float erreurAngle_precedente = 0;
float integrale_angle = 0;
unsigned long temps_precedent = 0;

// --- VARIABLES ÉTAT ---
bool objectifAtteint = false;
unsigned long tempsArrivePoint = 0;
const unsigned long PAUSE_ENTRE_POINTS = 1000;

// Variables pour le mode cercle
bool estModeCercle = false;
float rayonCercle = 0.0f;
static float vG_prev = 0, vD_prev = 0;

// --- Magnetometer Variables ---
float mag_offset_x = 0.0;
float mag_offset_y = 0.0;
float magnetic_north_angle = 0.0;
bool auto_calibrated = false;
bool debugMag = false;

const Point arrowparcours[] = {
  // SUPER SIMPLE ARROW - just basic lines
  
  {0.20, 0.00},   // Shaft: 10cm forward
  {0.20, 0.10},   // Left diagonal of head
  {0.20, -0.10},   // Tip of arrow
  {0.30, 0.0},  // Right diagonal of head
  {0.20, 0.10},   // Back to tip
};

// Update your NB_ARROW accordingly:
const int NB_ARROW = sizeof(arrowparcours) / sizeof(Point);

// Function to load the arrow path (add this near your loadBasePath function)
void loadArrowPath() {
  NB_POINTS = NB_ARROW;
  for (int i = 0; i < NB_POINTS; i++)
    PARCOURS[i] = arrowparcours[i];
  pointActuel = 0;
  parcoursFini = false;
  estModeCercle = false;  // Normal path mode
  
  SerialBT.println("🏹 Arrow path loaded");
  SerialBT.print("📏 Points: ");
  SerialBT.println(NB_POINTS);
  
  // Show the path for debugging
  SerialBT.println("📋 Arrow points:");
  for (int i = 0; i < NB_POINTS; i++) {
    SerialBT.print("  Point ");
    SerialBT.print(i + 1);
    SerialBT.print(": (");
    SerialBT.print(PARCOURS[i].x * 100, 1);
    SerialBT.print(", ");
    SerialBT.print(PARCOURS[i].y * 100, 1);
    SerialBT.println(") cm");
  }
}

// --- INTERRUPTIONS ENCODEURS ---
void IRAM_ATTR encoderG_A() {
  bool b = digitalRead(ENC_G_CH_B);
  ticksG += (b ? -1 : 1);
}

void IRAM_ATTR encoderD_A() {
  bool b = digitalRead(ENC_D_CH_B);
  ticksD += (b ? 1 : -1);
}

// --- FONCTIONS MOTEURS (avec moteurs montés à l'envers) ---
void setMoteurs(float vitesseGauche, float vitesseDroite) {
  vitesseGauche = constrain(vitesseGauche, -255, 255);
  vitesseDroite = constrain(vitesseDroite, -255, 255);

  // Moteur gauche (inversé)
  if (vitesseGauche >= 0) {
    digitalWrite(IN_1_G, HIGH);
    digitalWrite(IN_2_G, LOW);
  } else {
    digitalWrite(IN_1_G, LOW);
    digitalWrite(IN_2_G, HIGH);
    vitesseGauche = -vitesseGauche;
  }

  // Moteur droit (inversé)
  if (vitesseDroite >= 0) {
    digitalWrite(IN_1_D, LOW);
    digitalWrite(IN_2_D, HIGH);
  } else {
    digitalWrite(IN_1_D, HIGH);
    digitalWrite(IN_2_D, LOW);
    vitesseDroite = -vitesseDroite;
  }

  analogWrite(EN_G, vitesseGauche);
  analogWrite(EN_D, vitesseDroite);
}

void arreterMoteurs() {
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, LOW);
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, LOW);
  analogWrite(EN_G, 0);
  analogWrite(EN_D, 0);
}

// --- FONCTIONS UTILITAIRES ---
double normaliserAngle(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

// Obtenir la position actuelle du stylo
void getPositionStylo(double &x_stylo, double &y_stylo) {
  x_stylo = x_robot + OFFSET_STYLO * cos(theta_robot);
  y_stylo = y_robot + OFFSET_STYLO * sin(theta_robot);
}

// --- FONCTIONS PID ---
float calculerPID_Distance(float erreur, float dt) {
  float P = KP_DIST * erreur;
  
  integrale_dist += erreur * dt;
  integrale_dist = constrain(integrale_dist, -0.5, 0.5);
  float I = KI_DIST * integrale_dist;
  
  float derivee = (erreur - erreurDist_precedente) / dt;
  float D = KD_DIST * derivee;
  
  erreurDist_precedente = erreur;
  
  return P + I + D;
}

float calculerPID_Angle(float erreur, float dt) {
  erreur = normaliserAngle(erreur);
  
  float P = KP_ANGLE * erreur;
  
  integrale_angle += erreur * dt;
  integrale_angle = constrain(integrale_angle, -0.3, 0.3);
  float I = KI_ANGLE * integrale_angle;
  
  float derivee = normaliserAngle(erreur - erreurAngle_precedente) / dt;
  float D = KD_ANGLE * derivee;
  
  erreurAngle_precedente = erreur;
  
  return P + I + D;
}

// --- MISE À JOUR ODOMÉTRIE ---
void mettreAJourOdometrie() {
  noInterrupts();
  long currentG = ticksG;
  long currentD = ticksD;
  interrupts();
  
  long dG = currentG - lastTicksG;
  long dD = currentD - lastTicksD;
  
  if (dG == 0 && dD == 0) return;
  
  lastTicksG = currentG;
  lastTicksD = currentD;
  
  double distG = (dG * PERIMETRE) / TICKS_PAR_TOUR;
  double distD = (dD * PERIMETRE) / TICKS_PAR_TOUR;
  
  double dDist = (distG + distD) / 2.0;
  double dTheta = (distD - distG) / ENTRAXE;
  
  if (abs(dTheta) > 0.0001) {
    double R = dDist / dTheta;
    double dx = R * (sin(theta_robot + dTheta) - sin(theta_robot));
    double dy = R * (cos(theta_robot) - cos(theta_robot + dTheta));
    x_robot += dx;
    y_robot += dy;
  } else {
    x_robot += dDist * cos(theta_robot);
    y_robot += dDist * sin(theta_robot);
  }
  
  theta_robot = normaliserAngle(theta_robot + dTheta);
}

// --- FONCTION DE NAVIGATION OPTIMISÉE ---
void navigationMultiPoints() {
  if (parcoursFini) {
    arreterMoteurs();
    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    
    // Réinitialiser le mode cercle si on a terminé
    if (estModeCercle && parcoursFini) {
      estModeCercle = false;
    }
    return;
  }

  if (objectifAtteint) {
    // En mode cercle, ne pas s'arrêter complètement entre les points
    if (estModeCercle && pointActuel < NB_POINTS - 1) {
      // Passer directement au point suivant sans arrêt complet
      pointActuel++;
      objectifAtteint = false;
      integrale_dist = 0;
      integrale_angle = 0;
      
      SerialBT.print("➡️ Point suivant: (");
      SerialBT.print(PARCOURS[pointActuel].x * 100);
      SerialBT.print(", ");
      SerialBT.print(PARCOURS[pointActuel].y * 100);
      SerialBT.println(") cm");
    } else {
      // Comportement normal pour la séquence de base
      arreterMoteurs();
      digitalWrite(LEDU1, HIGH);
      digitalWrite(LEDU2, HIGH);
      if (millis() - tempsArrivePoint > PAUSE_ENTRE_POINTS) {
        if (pointActuel < NB_POINTS - 1) {
          pointActuel++;
          objectifAtteint = false;
          integrale_dist = integrale_angle = 0;
          SerialBT.print("🎯 Prochain point: (");
          SerialBT.print(PARCOURS[pointActuel].x * 100);
          SerialBT.print(", ");
          SerialBT.print(PARCOURS[pointActuel].y * 100);
          SerialBT.println(") cm");
        } else {
          parcoursFini = true;
          SerialBT.println("✅ Parcours terminé!");
        }
      }
    }
    return;
  }

  unsigned long now = millis();
  float dt = (now - temps_precedent) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  temps_precedent = now;

  double xs, ys;
  getPositionStylo(xs, ys);

  float tx = PARCOURS[pointActuel].x;
  float ty = PARCOURS[pointActuel].y;
  float dx = tx - xs;
  float dy = ty - ys;
  float dist = sqrt(dx*dx + dy*dy);
  
  // Tolérance adaptée pour les cercles
  float currentTolerance;
  if (estModeCercle) {
    // Tolérance plus grande pour les cercles
    currentTolerance = 0.04;  // 4cm pour les points du cercle
  } else {
    currentTolerance = DISTANCE_TOLERANCE;
  }

  if (dist < currentTolerance) {
    if (estModeCercle) {
      // En mode cercle, transition plus fluide
      objectifAtteint = true;
      tempsArrivePoint = now;
      
      // Ne pas arrêter les moteurs pour une transition fluide
      SerialBT.print("✓ Point atteint: (");
      SerialBT.print(tx * 100);
      SerialBT.print(", ");
      SerialBT.print(ty * 100);
      SerialBT.println(") cm");
    } else {
      // Comportement normal pour d'autres parcours
      arreterMoteurs();
      objectifAtteint = true;
      tempsArrivePoint = now;
      SerialBT.print("✅ Point atteint: (");
      SerialBT.print(tx * 100);
      SerialBT.print(", ");
      SerialBT.print(ty * 100);
      SerialBT.println(") cm");
    }
    return;
  }

  digitalWrite(LEDU1, (now / 500) % 2);
  digitalWrite(LEDU2, LOW);

  float angleCible = atan2(dy, dx);
  float errA = normaliserAngle(angleCible - theta_robot);
  
  // En mode cercle, limiter les corrections d'angle trop importantes
  if (estModeCercle) {
    // Limiter l'erreur d'angle pour éviter les rotations sur place
    float maxAngleCorrection = 0.3; // ~17 degrés max
    errA = constrain(errA, -maxAngleCorrection, maxAngleCorrection);
  }
  
  // Facteur de vitesse adapté selon le mode et les conditions
  float speedFactor;
  
  if (estModeCercle) {
    // Vitesse constante pour le cercle pour un mouvement fluide
    speedFactor = 0.6;  // 60% de la vitesse max
  } else {
    // Pour les autres parcours, vitesse adaptative standard
    if (dist < 0.03) {
      speedFactor = 0.4;
    } else if (dist < 0.06) {
      speedFactor = 0.6;
    } else {
      speedFactor = 0.8;
    }
    
    // Anti-oscillation: si l'erreur d'angle est grande
    if (abs(errA) > 0.5) {
      speedFactor *= (1.0 - abs(errA) / PI);
    }
  }
  
  // Calcul PID avec facteur de vitesse
  float cmdD = calculerPID_Distance(dist, dt) * speedFactor;
  float cmdA = calculerPID_Angle(errA, dt);
  
  // En mode cercle, réduire l'influence de l'angle
  if (estModeCercle) {
    cmdA *= 0.7;  // Réduire de 30% l'effet de la correction d'angle
  }

  // Assurer une vitesse minimale mais pas trop élevée
  if (abs(cmdD) < VITESSE_MIN && abs(cmdD) > 0) {
    cmdD = (cmdD > 0) ? VITESSE_MIN : -VITESSE_MIN;
  }
  
  // Limiter la vitesse maximale
  float maxSpeed;
  if (estModeCercle) {
    // Vitesse maximale très réduite pour les cercles
    maxSpeed = VITESSE_MAX * 0.6;  // 60% de la vitesse max pour les cercles
  } else {
    // Vitesse normale pour les autres parcours
    maxSpeed = (dist < 0.05) ? VITESSE_MAX * 0.5 : VITESSE_MAX * 0.8;
  }
  
  float vG = constrain(cmdD, -maxSpeed, maxSpeed) - cmdA;
  float vD = constrain(cmdD, -maxSpeed, maxSpeed) + cmdA;
  
  // Anti-oscillation: limiter les variations brusques de vitesse
  // Réduction significative pour les cercles
  float MAX_CHANGE = estModeCercle ? 15.0 : 30.0;
  
  vG = constrain(vG, vG_prev - MAX_CHANGE, vG_prev + MAX_CHANGE);
  vD = constrain(vD, vD_prev - MAX_CHANGE, vD_prev + MAX_CHANGE);
  
  vG_prev = vG;
  vD_prev = vD;
  
  setMoteurs(vG, vD);

  static unsigned long lastDbg = 0;
  if (now - lastDbg > 500) {
    lastDbg = now;
    SerialBT.print("📍 Stylo: (");
    SerialBT.print(xs * 100, 1);
    SerialBT.print(", ");
    SerialBT.print(ys * 100, 1);
    SerialBT.print(") → (");
    SerialBT.print(tx * 100, 1);
    SerialBT.print(", ");
    SerialBT.print(ty * 100, 1);
    SerialBT.print(") | Dist: ");
    SerialBT.print(dist * 100, 1);
    SerialBT.print("cm | Angle: ");
    SerialBT.print(errA * 180/PI, 1);
    SerialBT.println("°");
  }
}

// --- RESET ---
void resetParcours() {
  noInterrupts();
  ticksG = 0;
  ticksD = 0;
  lastTicksG = 0;
  lastTicksD = 0;
  interrupts();
  
  // Reset avec le stylo à l'origine
  x_robot = -OFFSET_STYLO;
  y_robot = 0.0;
  theta_robot = 0.0;
  
  pointActuel = 0;
  objectifAtteint = false;
  parcoursFini = false;
  
  integrale_dist = 0;
  integrale_angle = 0;
  erreurDist_precedente = 0;
  erreurAngle_precedente = 0;
  
  // Reset des variables de contrôle de vitesse
  vG_prev = 0;
  vD_prev = 0;
  
  SerialBT.println("📍 Reset - Stylo en (0,0)");
}

// --- DEBUG ---
void afficherDebug() {
  double x_stylo, y_stylo;
  getPositionStylo(x_stylo, y_stylo);
  
  SerialBT.println("\n=== DEBUG ===");
  SerialBT.print("Position stylo: (");
  SerialBT.print(x_stylo * 100, 1);
  SerialBT.print(", ");
  SerialBT.print(y_stylo * 100, 1);
  SerialBT.print(") cm | Angle: ");
  SerialBT.print(theta_robot * 180 / PI, 1);
  SerialBT.print("° | Point: ");
  SerialBT.print(pointActuel + 1);
  SerialBT.print("/");
  SerialBT.println(NB_POINTS);
}

void loadBasePath() {
  NB_POINTS = NB_BASE;
  for (int i = 0; i < NB_POINTS; i++)
    PARCOURS[i] = baseparcours[i];
  pointActuel = 0;
  parcoursFini = false;
  estModeCercle = false;  // Désactiver le mode cercle pour le parcours normal
}

// --- FONCTION OPTIMISÉE POUR GÉNÉRER UN CERCLE ---
// Fonction améliorée pour tracer un cercle entre 13 et 20 cm
void generateCirclePath(float radius_cm) {
  // Contraindre le rayon dans les limites acceptables
  radius_cm = constrain(radius_cm, 13.0f, 20.0f);
  float r = radius_cm / 100.0f;  // Convertir en mètres
  
  // Utiliser plus de points pour un mouvement plus fluide
  int numPoints = 16;  // Plus de points = moins de rotation entre les points
  NB_POINTS = numPoints;
  
  // Générer les points en commençant par le haut (90°)
  for (int i = 0; i < numPoints; i++) {
    float angle = 2.0f * PI * i / numPoints + PI/2;  // Commencer par le haut
    PARCOURS[i].x = r * cos(angle);
    PARCOURS[i].y = r * sin(angle);
  }
  
  // Reset complet avant de démarrer
  resetParcours();
  estModeCercle = true;  // Activer le mode cercle
  parcoursCharge = true;
  
  // Pour le debuggage
  SerialBT.print("🔄 Cercle fluide généré, R=");
  SerialBT.print(radius_cm);
  SerialBT.print(" cm — ");
  SerialBT.print(numPoints);
  SerialBT.println(" segments");
  
  // Afficher les points générés pour vérification
  SerialBT.println("Points du cercle:");
  for (int i = 0; i < numPoints; i++) {
    SerialBT.print("  Point ");
    SerialBT.print(i + 1);
    SerialBT.print(": (");
    SerialBT.print(PARCOURS[i].x * 100, 1);
    SerialBT.print(", ");
    SerialBT.print(PARCOURS[i].y * 100, 1);
    SerialBT.println(") cm");
  }
}

// --- MAGNETOMETER FUNCTIONS ---

void initMagnetometer() {
  Wire.begin(SDA, SCL);
  delay(100);

  // Basic magnetometer initialization
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x20); // CTRL_REG1_M
  Wire.write(0x90); // Medium performance, 10Hz
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x21); // CTRL_REG2_M
  Wire.write(0x00); // ±4 gauss scale
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x22); // CTRL_REG3_M
  Wire.write(0x00); // Continuous mode
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x23); // CTRL_REG4_M
  Wire.write(0x08); // Z high performance
  Wire.endTransmission();

  SerialBT.println("📡 Magnetometer initialized");
}

// Read raw magnetometer values
bool readRawMagnetometer(int16_t &x, int16_t &y) {
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x28);
  int error = Wire.endTransmission(false);

  if (error != 0) {
    if (debugMag) SerialBT.println("I2C error: " + String(error));
    return false;
  }

  Wire.requestFrom(ADDR_MAG, 4);
  if (Wire.available() < 4) {
    if (debugMag) SerialBT.println("Insufficient magnetometer data");
    return false;
  }

  uint8_t xLow = Wire.read();
  uint8_t xHigh = Wire.read();
  uint8_t yLow = Wire.read();
  uint8_t yHigh = Wire.read();

  x = (int16_t)((xHigh << 8) | xLow);
  y = (int16_t)((yHigh << 8) | yLow);

  return true;
}

// Check if magnetometer reading is valid
bool isValidMagReading(int16_t x, int16_t y) {
  return (abs(x) < 30000 && abs(y) < 30000 && (abs(x) > 10 || abs(y) > 10));
}

// Calculate heading from raw values
float calculateHeading(int16_t x, int16_t y) {
  // Apply calibration if available
  float x_cal = (float)x - mag_offset_x;
  float y_cal = (float)y - mag_offset_y;

  // Calculate heading
  float heading = atan2(y_cal, x_cal) * 180.0 / PI;

  // Normalize to 0-360°
  if (heading < 0) heading += 360.0;
  return heading;
}

// Get current heading relative to magnetic north
float getHeadingFromNorth() {
  int16_t x, y;
  if (!readRawMagnetometer(x, y) || !isValidMagReading(x, y)) {
    return -1;
  }

  float current_heading = calculateHeading(x, y);
  float heading_from_north = current_heading - magnetic_north_angle;

  // Normalize to 0-360°
  while (heading_from_north < 0) heading_from_north += 360.0;
  while (heading_from_north >= 360.0) heading_from_north -= 360.0;

  return heading_from_north;
}

// Save calibration to EEPROM
void saveMagCalibration() {
  EEPROM.write(0, AUTO_CALIB_FLAG);
  EEPROM.put(4, mag_offset_x);
  EEPROM.put(8, mag_offset_y);
  EEPROM.put(12, magnetic_north_angle);
  EEPROM.write(16, auto_calibrated ? 1 : 0);
  EEPROM.commit();
  SerialBT.println("🔒 Calibration saved to EEPROM");
}

// Load calibration from EEPROM
bool loadMagCalibration() {
  if (EEPROM.read(0) != AUTO_CALIB_FLAG) {
    return false;
  }
  
  EEPROM.get(4, mag_offset_x);
  EEPROM.get(8, mag_offset_y);
  EEPROM.get(12, magnetic_north_angle);
  auto_calibrated = (EEPROM.read(16) == 1);
  
  return auto_calibrated;
}

// Main calibration function using your existing motor control
void calibrateMagnetometer() {
  SerialBT.println("🧭 MAGNETOMETER CALIBRATION STARTING");
  SerialBT.println("Robot will rotate 360° to collect magnetic data...");
  delay(2000);

  // Reset position tracking
  noInterrupts();
  ticksG = 0;
  ticksD = 0;
  lastTicksG = 0;
  lastTicksD = 0;
  interrupts();

  // Arrays to store magnetometer data
  const int max_samples = 180;
  int16_t x_samples[max_samples];
  int16_t y_samples[max_samples];
  int sample_count = 0;

  // Track min/max for calibration
  int16_t x_min = 32767, x_max = -32768;
  int16_t y_min = 32767, y_max = -32768;

  // Calculate target rotation (approximately 360°)
  // Using your existing motor control system
  float target_rotation = 2 * PI; // Full circle in radians
  float current_rotation = 0;
  
  // Start slow rotation
  float rotation_speed = 80; // Slow speed for accurate data collection
  
  unsigned long start_time = millis();
  unsigned long last_sample_time = 0;

  while (sample_count < max_samples && current_rotation < target_rotation && (millis() - start_time) < 30000) {
    // Update odometry to track rotation
    mettreAJourOdometrie();
    current_rotation = abs(theta_robot);

    // Take sample every ~2 degrees
    if ((millis() - last_sample_time) > 100) {
      int16_t x, y;
      if (readRawMagnetometer(x, y) && isValidMagReading(x, y)) {
        x_samples[sample_count] = x;
        y_samples[sample_count] = y;

        // Update min/max for calibration
        x_min = min(x_min, x);
        x_max = max(x_max, x);
        y_min = min(y_min, y);
        y_max = max(y_max, y);

        sample_count++;
        last_sample_time = millis();

        // Progress feedback
        if (sample_count % 20 == 0) {
          SerialBT.print("📊 Samples: ");
          SerialBT.print(sample_count);
          SerialBT.print("/");
          SerialBT.print(max_samples);
          SerialBT.print(" | Rotation: ");
          SerialBT.print(current_rotation * 180 / PI, 0);
          SerialBT.println("°");
        }
      }
    }

    // Apply rotation using your motor system
    setMoteurs(-rotation_speed, rotation_speed); // Turn in place
    delay(20);
  }

  // Stop rotation
  arreterMoteurs();
  SerialBT.println("🛑 Rotation completed");

  // Calculate calibration offsets
  if (sample_count > 50) {
    mag_offset_x = (float)(x_max + x_min) / 2.0;
    mag_offset_y = (float)(y_max + y_min) / 2.0;

    SerialBT.println("📈 Calibration results:");
    SerialBT.print("   Samples: ");
    SerialBT.println(sample_count);
    SerialBT.print("   X range: ");
    SerialBT.print(x_min);
    SerialBT.print(" to ");
    SerialBT.print(x_max);
    SerialBT.print(" (offset: ");
    SerialBT.print(mag_offset_x, 1);
    SerialBT.println(")");
    SerialBT.print("   Y range: ");
    SerialBT.print(y_min);
    SerialBT.print(" to ");
    SerialBT.print(y_max);
    SerialBT.print(" (offset: ");
    SerialBT.print(mag_offset_y, 1);
    SerialBT.println(")");

    // Determine magnetic north reference
    delay(1000);
    SerialBT.println("🧭 Determining magnetic north reference...");
    
    // Take multiple readings to establish north reference
    float heading_sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 20; i++) {
      int16_t x, y;
      if (readRawMagnetometer(x, y) && isValidMagReading(x, y)) {
        float heading = calculateHeading(x, y);
        heading_sum += heading;
        valid_readings++;
      }
      delay(100);
    }

    if (valid_readings > 10) {
      magnetic_north_angle = heading_sum / valid_readings;
      auto_calibrated = true;
      
      SerialBT.print("✅ Calibration complete! North reference: ");
      SerialBT.print(magnetic_north_angle, 1);
      SerialBT.println("°");
      
      saveMagCalibration();
    } else {
      SerialBT.println("❌ Failed to establish north reference");
      auto_calibrated = false;
    }
  } else {
    SerialBT.println("❌ Insufficient samples for calibration");
    auto_calibrated = false;
  }

  // Reset robot position for your navigation system
  resetParcours();
}

// Turn to face magnetic north using your existing navigation
void faceNorth() {
  if (!auto_calibrated) {
    SerialBT.println("❌ Robot not calibrated. Use 'mag_calib' first.");
    return;
  }

  SerialBT.println("🧭 Turning to face magnetic north...");

  float current_heading = getHeadingFromNorth();
  if (current_heading < 0) {
    SerialBT.println("❌ Cannot read current heading");
    return;
  }

  SerialBT.print("📍 Current heading: ");
  SerialBT.print(current_heading, 1);
  SerialBT.println("° from north");

  // Calculate shortest turn to face north (0°)
  float turn_needed = -current_heading;
  
  // Normalize to shortest turn (-180 to +180)
  while (turn_needed > 180) turn_needed -= 360;
  while (turn_needed < -180) turn_needed += 360;

  if (abs(turn_needed) < 3.0) {
    SerialBT.println("✅ Already facing north (within 3° tolerance)");
    return;
  }

  SerialBT.print("🔄 Turn needed: ");
  SerialBT.print(turn_needed, 1);
  SerialBT.println("°");

  // Perform turn using magnetometer feedback
  float target_heading = 0.0; // North
  float tolerance = 3.0; // 3 degree tolerance
  
  unsigned long start_time = millis();
  unsigned long timeout = 15000; // 15 second timeout
  
  while ((millis() - start_time) < timeout) {
    float current = getHeadingFromNorth();
    if (current < 0) continue; // Skip invalid readings

    float error = target_heading - current;
    
    // Handle wrap-around (e.g., from 359° to 1°)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) <= tolerance) {
      SerialBT.println("✅ Target reached!");
      break;
    }

    // Calculate motor speeds based on error
    float turn_speed = constrain(abs(error) * 2.0, 60, 150);
    
    if (error > 0) {
      // Turn left (counterclockwise)
      setMoteurs(-turn_speed, turn_speed);
    } else {
      // Turn right (clockwise)
      setMoteurs(turn_speed, -turn_speed);
    }

    delay(50);
  }

  arreterMoteurs();
  delay(500);

  // Verify final heading
  float final_heading = getHeadingFromNorth();
  if (final_heading >= 0) {
    SerialBT.print("🎯 Final heading: ");
    SerialBT.print(final_heading, 1);
    SerialBT.println("° from north");
    
    if (abs(final_heading) <= 5.0 || abs(final_heading - 360) <= 5.0) {
      SerialBT.println("✅ SUCCESS: Now facing magnetic north!");
    } else {
      SerialBT.println("⚠️ Close but not perfect - within acceptable range");
    }
  }
}

// Get magnetometer status
void showMagStatus() {
  SerialBT.println("\n🧭 === MAGNETOMETER STATUS ===");
  SerialBT.print("Calibrated: ");
  SerialBT.println(auto_calibrated ? "YES ✅" : "NO ❌");
  
  if (auto_calibrated) {
    SerialBT.print("Offsets: X=");
    SerialBT.print(mag_offset_x, 1);
    SerialBT.print(", Y=");
    SerialBT.println(mag_offset_y, 1);
    SerialBT.print("North reference: ");
    SerialBT.print(magnetic_north_angle, 1);
    SerialBT.println("°");
    
    float current = getHeadingFromNorth();
    if (current >= 0) {
      SerialBT.print("Current heading: ");
      SerialBT.print(current, 1);
      SerialBT.println("° from north");
    }
  } else {
    SerialBT.println("Use 'mag_calib' to calibrate");
  }
}

// Complete arrow function using the clean approach
void drawNorthArrowFixed() {
  /*if (!auto_calibrated) {
    SerialBT.println("❌ Magnetometer not calibrated. Use 'mag_calib' first.");
    return;
  }*/
  
  SerialBT.println("🧭 Drawing north-pointing arrow (8cm)...");
  
  // First, face north
  //faceNorth();
  delay(1000);
  
  // Reset position coordinates without moving the robot
  SerialBT.println("📍 Resetting position to (0,0) without moving...");
  noInterrupts();
  ticksG = 0;
  ticksD = 0;
  lastTicksG = 0;
  lastTicksD = 0;
  interrupts();
  
  // Reset robot position coordinates (pen at origin)
  x_robot = -OFFSET_STYLO;
  y_robot = 0.0;
  theta_robot = 0.0;
  
  // Reset PID variables to avoid interference
  integrale_dist = 0;
  integrale_angle = 0;
  erreurDist_precedente = 0;
  erreurAngle_precedente = 0;
  vG_prev = 0;
  vD_prev = 0;
  
  // Load the predefined arrow path
  loadArrowPath();
  
  // Set course as loaded
  parcoursCharge = true;
  
  SerialBT.println("✅ Ready to draw arrow!");
}

void setup() {
  // Initialisation USB (optionnel) et Bluetooth
  Serial.begin(115200);
  SerialBT.begin("DrawBot");
  SerialBT.println("🤖 DrawBot prêt – en attente de commande BT…");

  // Configuration des pins
  pinMode(EN_G, OUTPUT);
  pinMode(EN_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);
  
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_D_CH_B, INPUT_PULLUP);

  // Attachement des interruptions
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), encoderG_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), encoderD_A, RISING);

  // Activation des drivers moteurs
  digitalWrite(EN_G, HIGH);
  digitalWrite(EN_D, HIGH);
  arreterMoteurs();

  // Initialisation du timer pour le PID
  temps_precedent = millis();

  // Affichage des commandes Bluetooth disponibles
  SerialBT.println("\nCommandes BT :");
  SerialBT.println("  base       -> charger le parcours de base");
  SerialBT.println("  circle R   -> tracer un cercle de rayon R cm (13–20)");
  SerialBT.println("  r          -> reset parcours");
  SerialBT.println("  d          -> debug position");
  SerialBT.println("\nEnvoie ta commande pour démarrer.");

  // Petite animation LED pour indiquer que c'est prêt
  for (int i = 0; i < 6; i++) {
    digitalWrite(LEDU1, i % 2);
    digitalWrite(LEDU2, (i + 1) % 2);
    delay(200);  // Animation plus rapide
  }
  // LEDs éteintes en fin de setup
  arreterMoteurs();
  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  // Initialize EEPROM for magnetometer calibration
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize magnetometer
  initMagnetometer();
  
  // Try to load previous calibration
  if (loadMagCalibration()) {
    SerialBT.println("🧭 Previous magnetometer calibration loaded");
  }
  
  // Add magnetometer commands to your help text
 // In your setup() function, update the command list:
SerialBT.println("\nCommandes BT :");
SerialBT.println("  base         -> charger le parcours de base");
SerialBT.println("  circle R     -> tracer un cercle de rayon R cm (13–20)");
SerialBT.println("  mag_calib    -> calibrer le magnétomètre");
SerialBT.println("  face_north   -> orienter vers le nord magnétique");
SerialBT.println("  arrow [L]    -> dessiner flèche nord (L=longueur en cm)");
SerialBT.println("  simple_arrow [L] -> dessiner ligne nord simple");
SerialBT.println("  mag_status   -> statut du magnétomètre");
SerialBT.println("  r            -> reset parcours");
SerialBT.println("  d            -> debug position");
}


void loop() {
  // 1) Lecture des commandes Bluetooth
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');

    // Charger le parcours de base
    if (cmd == "base") {
      loadBasePath();
      parcoursCharge = true;
      SerialBT.println("🔄 Parcours de base chargé");
    }
    // Générer un cercle de rayon R cm
    else if (cmd.startsWith("circle")) {
      float r;
      if (sscanf(cmd.c_str() + 7, "%f", &r) == 1) {
        generateCirclePath(r);
        parcoursCharge = true;
        SerialBT.print("🔄 Cercle généré, r=");
        SerialBT.print(r);
        SerialBT.println(" cm");
      } else {
        SerialBT.println("⚠️ Syntaxe: circle <rayon_cm>");
      }
    }
    // Reset complet du parcours actuel
    else if (cmd == "r" || cmd == "R") {
      resetParcours();
      estModeCercle = false;  // Désactiver explicitement le mode cercle
      parcoursCharge = true;  // on peut relancer le même parcours
      SerialBT.println("📍 Reset effectué");
    }
    // Afficher la debug actuelle
    else if (cmd == "d" || cmd == "D") {
      afficherDebug();
    }
     // New magnetometer commands
  else if (cmd == "mag_calib") {
    calibrateMagnetometer();
    parcoursCharge = false; // Will need to reload a path after calibration
  }
  else if (cmd == "face_north") {
    faceNorth();
  }
  else if (cmd == "mag_status") {
    showMagStatus();
  }
  // In your loop() function, replace the arrow commands with:
// In your loop() function, replace your arrow command with:
else if (cmd.startsWith("arrow")) {
  drawNorthArrowFixed();
  SerialBT.println("🏹 North arrow (8cm) ready");
}
  
    // Commande inconnue
    else {
      SerialBT.print("❓ Cmd inconnue: ");
      SerialBT.println(cmd);
    }
  }

  // 2) Si aucun parcours n'est chargé, on reste en veille
  if (!parcoursCharge) {
    delay(50);
    return;
  }

  // 3) Dès qu'on a un parcours, on met à jour l'odométrie…
  mettreAJourOdometrie();

  // … et on lance la navigation
  navigationMultiPoints();

  // 4) Petite pause pour stabilité
  delay(20);
}