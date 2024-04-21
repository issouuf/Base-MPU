#include <Arduino.h>
#include <String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Définir les pins des moteurs
const int moteurGauchePin1 = 26; // IN1
const int moteurGauchePin2 = 27; // IN2
const int moteurDroitPin1 = 12;  // IN3
const int moteurDroitPin2 = 14;  // IN4

// Définir les canaux PWM
const int channelGauche1 = 0;
const int channelGauche2 = 1;
const int channelDroit1 = 2;
const int channelDroit2 = 3;

// LED ROUGE
const int ledRouge = 23;

// Définir la fréquence et la résolution PWM
const int freq = 20000;   // Fréquence en Hz
const int resolution = 8; // Résolution en bits

// Mesure tension batterie
#define pinBatterie 34

// Définir les pins encodeurs
#define CLKD 17 // CLK ENCODER droit
#define DTD 16  // DT ENCODER gauche
#define CLKG 19 // CLK ENCODER gauche
#define DTG 18  // DT ENCODER gauche

ESP32Encoder encoderD;
ESP32Encoder encoderG;

Adafruit_MPU6050 mpu;       // Création d'un objet mpu de type Adafruit_MPU6050 pour interagir avec le capteur MPU6050
sensors_event_t a, g, temp; // Déclaration des variables a, g et temp de type sensors_event_t pour stocker les données du MPU6050

float accx;
float accy;
float accz;

char FlagCalcul = 0;
float Te = 10;    // période d'échantillonage en ms
float Tau = 1000; // constante de temps du filtre en ms
float Tauv = 200;

float thetaG, thetaV, thetaVF, angleGyro, thetaGF, angle;

float thetaeq = 0.044; // 0
float consigne = 0 - thetaeq;

float erreur;
float vitesse;
float commande;
float ec;
float gyro;

float offsetMPU = 0.95; // 0.95

float frt = 40.2; // 0

// Définir les variables pour l'asservissement de vitesse
//  Ajoutez ces variables globales
float vitesseConsigne = 0; // choix de la vitesse de consigne
// float crtlvit;
float positionActuelle = 0;

// float vitesseConsigne = crtlvit;
float derniereErreur = 0;
float integrale = 0;
float vitesseFiltree = 0;
float positionPrecedente = 0;
float vitesseActuelle = 0;
float sommeErreurVitesse = 0;
// réglage PID
float Kp_vitesse = 0.015;
float Ki_vitesse = 0.05;
float Kd_vitesse = 2;
// stockage du temps de la dernière mesure
unsigned long dernierTemps = 0;

float commandeVitesse;

// incrément de la rampe pour la vitesse
float increment = 0.005;

int direction; // direction

int slider;

// contrôle de la batterie
float valeurBrute;
float seuilCritique = 6.5; // seuil critique pour la batterie
float tensionBatterie;     // tension de la batterie
char flagBatterie = 0;     // flag pour la batterie

// coefficient du filtre
float A, B;
float Avit, Bvit;

float Kp = 1276;
float Kd = 28.4;

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {

    if (tensionBatterie < seuilCritique)
    {
      // Allumer la LED rouge
      digitalWrite(ledRouge, HIGH);
      flagBatterie = 1;
    }
    else
    {
      // Éteindre la LED rouge
      digitalWrite(ledRouge, LOW);
      flagBatterie = 0;
    }

    mpu.getEvent(&a, &g, &temp); // Récupérez les données du MPU6050

    // récupérer les données d'accélération
    accx = a.acceleration.x;
    accy = a.acceleration.y;
    accz = a.acceleration.z;

    // Calculez l'angle ici

    // Calculez l'angle à partir des données d'accélération
    thetaG = atan2(a.acceleration.y, a.acceleration.x);

    // Filtrez les deux angles pour obtenir un angle précis
    thetaGF = A * (thetaG + B * thetaGF); // filtre passe bas

    // Calculez l'angle à partir des données de vitesse
    thetaV = -(g.gyro.z) * Tau / 1000;

    thetaVF = A * (thetaV + B * thetaVF); // filtre passe haut

    angle = thetaGF + thetaVF; // angle final

    erreur = (consigne - angle);

    // commande = (erreur * Kp) + (Kd*(g.gyro.z));

    // calcul de la vitesse à parir des encodeurs
    long dernierePositionD = 0;
    long dernierePositionG = 0;
    long nouvellePositionD, nouvellePositionG;
    // unsigned long tempsActuel = millis(); // temps actuel en millisecondes
    // float dt = (tempsActuel - dernierTemps) / 1000.0; // Calcule du temps écoulé en secondes

    // lecture des encodeurs

    nouvellePositionD = encoderD.getCount();
    nouvellePositionG = encoderG.getCount();

    // positionActuelle = ((nouvellePositionD - dernierePositionD) + (nouvellePositionG - dernierePositionG))/2;

    positionActuelle = (nouvellePositionD + nouvellePositionG) / 2;
    vitesseActuelle = (positionActuelle - positionPrecedente);
    positionPrecedente = positionActuelle;

    // filtre passe bas sur la mesure de la vitesse
    vitesseFiltree = Avit * (vitesseActuelle + Bvit * vitesseFiltree);

    dernierePositionD = nouvellePositionD;
    dernierePositionG = nouvellePositionG;

    float erreurVitesse = vitesseConsigne - vitesseFiltree;

    float deltaErreur = erreurVitesse - derniereErreur;
    derniereErreur = erreurVitesse;

    commandeVitesse = Kp_vitesse * erreurVitesse + (Kd_vitesse * deltaErreur) / Te;
    if (commandeVitesse > 0.1)
    {
      commandeVitesse = 0.1;
    }
    else if (commandeVitesse < -0.1)
    {
      commandeVitesse = -0.1;
    }

    consigne = commandeVitesse - thetaeq;

    float commande = (erreur * Kp) + (Kd * (g.gyro.z));

    if (commande >= 0)
    {
      commande = commande + frt; //-60
    }
    else if (commande <= 0)
    {
      commande = commande - frt; //+60
    }

    if (commande >= 120)
    {
      commande = 120;
    }
    else if (commande <= -120)
    {
      commande = -120;
    }

    if (vitesseFiltree < vitesseConsigne)
    {
      vitesseFiltree += increment;
    }
    // Si la vitesse actuelle est supérieure à la vitesse consignée, diminuer la vitesse actuelle
    else if (vitesseFiltree > vitesseConsigne)
    {
      vitesseFiltree -= increment;
    }

    if (flagBatterie == 1)
    {
      ledcWrite(channelGauche2, 0); // IN1
      ledcWrite(channelGauche1, 0); // IN2
      ledcWrite(channelDroit1, 0);  // IN3
      ledcWrite(channelDroit2, 0);  // IN4
    }
    else if (flagBatterie == 0)
    {
      ledcWrite(channelGauche2, 127 - commande + direction); // IN1
      ledcWrite(channelGauche1, 127 + commande - direction); // IN2
      ledcWrite(channelDroit1, 127 + commande + direction);  // IN3
      ledcWrite(channelDroit2, 127 - commande - direction);  // IN4
    }

    // ledcWrite(channelGauche2, 127 - commande + direction); // IN1
    // ledcWrite(channelGauche1, 127 + commande - direction); // IN2
    // ledcWrite(channelDroit1, 127 + commande + direction);  // IN3
    // ledcWrite(channelDroit2, 127 - commande - direction);  // IN4

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32U-A");
  // Configurer les pins et les canaux PWM
  ledcSetup(channelGauche1, freq, resolution);
  ledcSetup(channelGauche2, freq, resolution);
  ledcSetup(channelDroit1, freq, resolution);
  ledcSetup(channelDroit2, freq, resolution);

  ledcAttachPin(moteurGauchePin1, channelGauche1);
  ledcAttachPin(moteurGauchePin2, channelGauche2);
  ledcAttachPin(moteurDroitPin1, channelDroit1);
  ledcAttachPin(moteurDroitPin2, channelDroit2);

  pinMode(ledRouge, OUTPUT);

  pinMode(pinBatterie, INPUT);

  // initialisation des encodeurs
  encoderD.attachHalfQuad(DTD, CLKD);
  encoderD.setCount(0);
  encoderG.attachHalfQuad(DTG, CLKG);
  encoderG.setCount(0);

  Serial.printf("Bonjour \n\r");

  if (!mpu.begin())
  { // Initialisez le MPU6050
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de créer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te;

  Avit = 1 / (1 + Tauv / Te);
  Bvit = Tauv / Te;

  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
    }
    if (commande == "Kp")
    {
      Kp = valeur.toFloat();
    }
    if (commande == "Kd")
    {
      Kd = valeur.toFloat();
    }
    if (commande == "thetaeq")
    {
      thetaeq = valeur.toFloat();
    }
    if (commande == "frt")
    {
      frt = valeur.toFloat();
    }
    if (commande == "kpv")
    {
      Kp_vitesse = valeur.toFloat();
    }
    if (commande == "kiv")
    {
      Ki_vitesse = valeur.toFloat();
    }
    if (commande == "kdv")
    {
      Kd_vitesse = valeur.toFloat();
    }
    if (commande == "Tauv")
    {
      Tauv = valeur.toFloat();
    }
    if (commande == "crtl")
    {
      vitesseConsigne = valeur.toFloat();
    }
    if (commande == "D")
    {
      direction = valeur.toFloat();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void receptionBT(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  Serial.printf("%c", ch);
  if ((ch == 13) or (ch == 10) or (ch == '#'))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "A")
    {
      vitesseConsigne = 5;
    }
    if (commande == "R")
    {
      vitesseConsigne = -5;
    }
    if (commande == "S")
    {
      vitesseConsigne = 0;
    }
    if (commande == "T")
    {
      vitesseConsigne = valeur.toInt();
    }
    if (commande == "D")
    {
      direction = 1.2 * valeur.toInt();
    }
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{

  while (SerialBT.available() > 0) // tant qu'il y a des caractères à lire
  {
    receptionBT(SerialBT.read());
  }

  // Lire la valeur de tension de la batterie
  valeurBrute = analogRead(pinBatterie);
  tensionBatterie = (valeurBrute / 4095.0) * 3.23;

  // Convertir la tension du pont diviseur en tension de la batterie
  tensionBatterie = tensionBatterie * (7.2 / 3.23);

  Serial.printf("Tension de la batterie : %2f V   Valeur brute: %d \n", tensionBatterie, valeurBrute);
  SerialBT.printf("*B%d\n", int(tensionBatterie * 10));

  // // Vérifier si la tension de la batterie est critique
  // if (tensionBatterie < seuilCritique) {
  //     // Allumer la LED rouge
  //     digitalWrite(ledRouge, HIGH);
  // } else {
  //     // Éteindre la LED rouge
  //     digitalWrite(ledRouge, LOW);
  // }

  if (FlagCalcul == 1)
  {
    // Serial.printf("%f %f %f %f \n", thetaG,thetaGF,thetaVF,  angle); //affichage sur le moniteur série des données filtrées
    // Serial.printf("%f %f %f %f\n", erreur, commande); // affichage de l'angle sur le moniteur série
    // Serial.printf("%f %f\n",positionActuelle,vitesseFiltree);
    // Serial.printf("%f",vitesseActuelle);
    // Serial.printf("  %f",vitesseFiltree);
    // Serial.printf("    %f",angle);
    // Serial.printf("      %f\n",direction);
    FlagCalcul = 0;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}

// sauvegarde du programme

// #include <Arduino.h>
// #include <String.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <ESP32Encoder.h>

// // Définir les pins des moteurs
// const int moteurGauchePin1 = 26; // IN1
// const int moteurGauchePin2 = 27; // IN2
// const int moteurDroitPin1 = 12;  // IN3
// const int moteurDroitPin2 = 14;  // IN4

// // Définir les canaux PWM
// const int channelGauche1 = 0;
// const int channelGauche2 = 1;
// const int channelDroit1 = 2;
// const int channelDroit2 = 3;

// // Définir la fréquence et la résolution PWM
// const int freq = 20000;   // Fréquence en Hz
// const int resolution = 8; // Résolution en bits

// //Définir les pins encodeurs
// #define CLKD 17 // CLK ENCODER droit
// #define DTD 16// DT ENCODER gauche
// #define CLKG 18 // CLK ENCODER gauche
// #define DTG 19// DT ENCODER gauche

// ESP32Encoder encoderD;
// ESP32Encoder encoderG;

// Adafruit_MPU6050 mpu;       // Création d'un objet mpu de type Adafruit_MPU6050 pour interagir avec le capteur MPU6050
// sensors_event_t a, g, temp; // Déclaration des variables a, g et temp de type sensors_event_t pour stocker les données du MPU6050

// float accx;
// float accy;
// float accz;

// char FlagCalcul = 0;
// float Te = 10;    // période d'échantillonage en ms
// float Tau = 1000; // constante de temps du filtre en ms

// float thetaG, thetaV, thetaVF, angleGyro, thetaGF, angle;

// float thetaeq = 0;
// float consigne = 0 - thetaeq;
// float erreur;
// float vitesse;
// float aplus;
// float amoins;
// float commande;
// float ec;
// float gyro;

// float offsetMPU= 0.95; //0.95

// float frt=0;

// // coefficient du filtre
// float A, B;

// float Kp = 500;
// float Kd = 1;
// void controle(void *parameters)
// {
//   TickType_t xLastWakeTime;
//   xLastWakeTime = xTaskGetTickCount();
//   while (1)
//   {

//     mpu.getEvent(&a, &g, &temp); // Récupérez les données du MPU6050

//     // récupérer les données d'accélération
//     accx = a.acceleration.x;
//     accy = a.acceleration.y;
//     accz = a.acceleration.z;

//     // Calculez l'angle ici

//     // Calculez l'angle à partir des données d'accélération
//     thetaG = atan2(a.acceleration.y, a.acceleration.x);

//     // Filtrez les deux angles pour obtenir un angle précis
//     thetaGF = A * (thetaG + B * thetaGF); // filtre passe bas

//     // Calculez l'angle à partir des données de vitesse
//     thetaV = -(g.gyro.z) * Tau / 1000;

//     thetaVF = A * (thetaV + B * thetaVF); // filtre passe haut

//     angle = thetaGF + thetaVF; // angle final

//     erreur = (consigne - angle);

//     commande = (erreur * Kp) + (Kd*(g.gyro.z));

//     if (commande >= 0)
//     {
//       commande = commande + frt; //-60
//     }
//     else if (commande <= 0)
//     {
//       commande = commande - frt; //+60
//     }

//     if (commande >= 120)
//     {
//       commande = 120;
//     }
//     else if (commande <= -120)
//     {
//       commande = -120;
//     }

//     ledcWrite(channelGauche1, 127 + commande); // IN1
//     ledcWrite(channelGauche2, 127 - commande); // IN2
//     ledcWrite(channelDroit1, 127 + commande);  // IN3
//     ledcWrite(channelDroit2, 127 - commande);  // IN4

//     //lecture des encodeurs
//     long newPositionD = -encoderD.getCount() / 2;
//     long newPositionG = encoderG.getCount() / 2;

//     FlagCalcul = 1;
//     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
//   }
// }

// void setup()
// {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   // Configurer les pins et les canaux PWM
//   ledcSetup(channelGauche1, freq, resolution);
//   ledcSetup(channelGauche2, freq, resolution);
//   ledcSetup(channelDroit1, freq, resolution);
//   ledcSetup(channelDroit2, freq, resolution);

//   ledcAttachPin(moteurGauchePin1, channelGauche1);
//   ledcAttachPin(moteurGauchePin2, channelGauche2);
//   ledcAttachPin(moteurDroitPin1, channelDroit1);
//   ledcAttachPin(moteurDroitPin2, channelDroit2);

//   //initialisation des encodeurs
//   encoderD.attachHalfQuad ( DTD, CLKD );
//   encoderD.setCount ( 0 );
//   encoderG.attachHalfQuad ( DTG, CLKG );
//   encoderG.setCount ( 0 );

//   Serial.printf("Bonjour \n\r");

//   if (!mpu.begin())
//   { // Initialisez le MPU6050
//     Serial.println("Failed to find MPU6050 chip");
//     while (1)
//     {
//       delay(10);
//     }
//   }

//   xTaskCreate(
//       controle,   // nom de la fonction
//       "controle", // nom de la tache que nous venons de créer
//       10000,      // taille de la pile en octet
//       NULL,       // parametre
//       10,         // tres haut niveau de priorite
//       NULL        // descripteur
//   );

//   // calcul coeff filtre
//   A = 1 / (1 + Tau / Te);
//   B = Tau / Te;

//   //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
// }

// void reception(char ch)
// {

//   static int i = 0;
//   static String chaine = "";
//   String commande;
//   String valeur;
//   int index, length;

//   if ((ch == 13) or (ch == 10))
//   {
//     index = chaine.indexOf(' ');
//     length = chaine.length();
//     if (index == -1)
//     {
//       commande = chaine;
//       valeur = "";
//     }
//     else
//     {
//       commande = chaine.substring(0, index);
//       valeur = chaine.substring(index + 1, length);
//     }

//     if (commande == "Tau")
//     {
//       Tau = valeur.toFloat();
//       // calcul coeff filtre
//       A = 1 / (1 + Tau / Te);
//       B = Tau / Te * A;
//     }
//     if (commande == "Te")
//     {
//       Te = valeur.toInt();
//     }
//     if (commande == "Kp")
//     {
//       Kp = valeur.toFloat();
//     }
//     if (commande == "Kd")
//     {
//       Kd = valeur.toFloat();
//     }
//     if (commande == "thetaeq")
//     {
//       thetaeq = valeur.toFloat();
//     }
//     if (commande == "frt") {
//       frt = valeur.toFloat();
//     }

//     chaine = "";
//   }
//   else
//   {
//     chaine += ch;
//   }
// }

// void loop()
// {

//   // if (erreur <=0) {
//   //   erreur2 = erreur* -120;
//   // }else {
//   //   erreur2 = erreur*120;
//   // }

//   // commande = (erreur*Kp)+(g.gyro.z*-1*Kd);

//   // if (commande >= 0){
//   //   commande = commande+100;
//   // }else {
//   //   commande = commande-100;
//   // }

//   aplus = 127 + commande;
//   amoins = 127 - commande;

//   vitesse = round(erreur * (aplus + amoins));

//   if (vitesse > 255)
//   {
//     vitesse = 255;
//   }
//   else if (vitesse < -255)
//   {
//     vitesse = -255;
//   }

//   if (FlagCalcul == 1)
//   {
//     // Serial.printf("%f %f %f %f \n", thetaG,thetaGF,thetaVF,  angle); //affichage sur le moniteur série des données filtrées
//     Serial.printf("%f %f %f %f\n", erreur, commande); // affichage de l'angle sur le moniteur série
//    // Serial.printf("%f \n",g.gyro.z);
//     FlagCalcul = 0;
//   }
// }

// void serialEvent()
// {
//   while (Serial.available() > 0) // tant qu'il y a des caractères à lire
//   {
//     reception(Serial.read());
//   }
// }
