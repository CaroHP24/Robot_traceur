#include <SimpleTimer.h>

// asservissement en position angulaire un moteur à courant continu.

SimpleTimer timer;  // Timer pour échantillonnage

//Moteur A
const int MoteurA1 = 12;     // Commande de sens moteur, Input 1
const int MoteurA2 = 13;     // Commande de sens moteur, Input 21
const int pinPowerA = 9;     // Commande de vitesse moteur, Output Enabled1
const int encoderPinA1 = 3;  // compteur 1
const int encoderPinA2 = 5;  // compteur 2
//definition des entrées
const int MoteurB1 = 8;      // Commande de sens moteur, Input 1
const int MoteurB2 = 7;      // Commande de sens moteur, Input 2
const int pinPowerB = 6;     // Commande de vitesse moteur, Output Enabled1
const int encoderPinB1 = 2;  // compteur 1
const int encoderPinB2 = 4;  // compteur 2
bool rotationMG_1 =1;
bool rotationMG_2 =0;
bool rotationMD_1 =1;
bool rotationMD_2 =0;
//init echantillonage
unsigned int timeA = 0;
unsigned int timeB = 0;
const int frequence_echantillonnage = 20;

//init compteur :
int encoder0PosA = 0;  //position de départ=0
boolean A1_set = false;
//init compteur :
int encoder0PosB = 0;  //position de départ=0
boolean B1_set = false;


//consigne
double target_cm = 20;
double target_deg = 18 * target_cm;
int target_ticks;  //plus simple d'asservir en ticks car ce sera toujours un nombre entier

// init calculs asservissement PID
int erreurA = 0;  //erreur
float erreurPrecedenteA = 0;
float somme_erreurA = 0;
// init calculs asservissement PID
int erreurB = 0;  //erreur
float erreurPrecedenteB = 0;
float somme_erreurB = 0;
unsigned long temps;

//Definition des constantes du correcteur PID
const float kpA = 0.8;  // Coefficient proportionnel (choisis par essais successifs)
const float kiA = 0;    // Coefficient intégrateur
const float kdA = 0;    // Coefficient dérivateur
//Definition des constantes du correcteur PID
const float kpB = 4;  // Coefficient proportionnel (choisis par essais successifs)
const float kiB = 0;  // Coefficient intégrateur
const float kdB = 0;  // Coefficient dérivateur

int n=0;
/* Routine d'initialisation */
void setup() {
  target_ticks = target_deg * (120.0 * 16.0) / 360.0;
  Serial.begin(115200);  // Initialisation port COM
  //MOTEURA
  pinMode(pinPowerA, OUTPUT);  // Sorties commande moteur
  pinMode(MoteurA1, OUTPUT);
  pinMode(MoteurA2, OUTPUT);
  pinMode(encoderPinA1, INPUT);  //sorties encodeur
  pinMode(encoderPinA2, INPUT);
  digitalWrite(encoderPinA1, HIGH);  // Resistance interne arduino ON
  digitalWrite(encoderPinA2, HIGH);  // Resistance interne arduino ON
  // Interruption de l'encodeur A en sortie 0 (pin 2)

  //moteurB
  pinMode(pinPowerB, OUTPUT);  // Sorties commande moteur
  pinMode(MoteurB1, OUTPUT);
  pinMode(MoteurB2, OUTPUT);
  pinMode(encoderPinB1, INPUT);  //sorties encodeur
  pinMode(encoderPinB2, INPUT);
  digitalWrite(encoderPinB1, HIGH);  // Resistance interne arduino ON
  digitalWrite(encoderPinB2, HIGH);  // Resistance interne arduino ON


  //MOTEURA
  attachInterrupt(0, doEncoderA1, CHANGE);
  // Interruption de l'encodeur A en sortie 1 (pin 3)ç
  analogWrite(pinPowerA, 0);  // Initialisation sortie moteur à 0
  delay(300);                 // Pause de 0,3 sec pour laisser le temps au moteur de s'arréter si celui-ci est en marche
  // Interruption pour calcul du PID et asservissement appelée toutes les 10ms
  //MOTEURB
  attachInterrupt(0, doEncoderB1, CHANGE);
  // Interruption de l'encodeur A en sortie 1 (pin 3)
  analogWrite(pinPowerB, 0);  // Initialisation sortie moteur à 0
  delay(300);                 // Pause de 0,3 sec pour laisser le temps au moteur de s'arréter si celui-ci est en marche
                              // Interruption pour calcul du PID et asservissement appelée toutes les 10ms

  timer.setInterval(1000 / frequence_echantillonnage, asservissement);
}



/* Fonction principale */
void loop() {
  timer.run();
  /*
  if(n==1 or n==4 or n==7)
  {
    n++;
    triangle();
  }
  */
  if(n==1 or n==4 or n==7 or n==10)
  {
    n++;
    carre();
  }
  
}

//---- Cette fonction est appelée toutes les 20ms pour calcul du correcteur PID
void asservissement() 
{
  timeA += 20;  // pratique pour graphes excel après affichage sur le moniteur
  erreurA = target_ticks - encoder0PosA;
  timeB += 20;  // pratique pour graphes excel après affichage sur le moniteur
  erreurB = target_ticks - encoder0PosB;

  somme_erreurA += erreurA;
  somme_erreurB += erreurB;

  // Calcul de la vitesse courante du moteur
  int vitMoteurA = kpA * erreurA + kdA * (erreurA - erreurPrecedenteA) + kiA * (somme_erreurA);
  erreurPrecedenteA = erreurA;  // Ecrase l'erreur précedente par la nouvelle erreur
  // Normalisation et contrôle du moteur
  int vitMoteurB = kpB * erreurB + kdB * (erreurB - erreurPrecedenteB) + kiB * (somme_erreurB);
  erreurPrecedenteB = erreurB;

  if (vitMoteurA > 255) vitMoteurA = 80; 
  else if (vitMoteurA < -255) vitMoteurA = 80;
  if (vitMoteurB > 255) vitMoteurB = 80;  
  else if (vitMoteurB < -255) vitMoteurB = 80;

  TournerA(vitMoteurA,rotationMD_1,rotationMD_2);        //DROITE
  TournerB(vitMoteurB,rotationMG_1,rotationMG_2);        //GAUCHE
  float angle_degA = encoder0PosA / 120.0 / 8 * 360; 
  float distanceA = encoder0PosA / 120.0 / 16*360 / 17.90;
  float angle_degB = encoder0PosB / 120.0 / 8.0 * 360;  
  float distanceB = encoder0PosB / 120.0 / 16.0 * 360 / 17.90;
  
  if(distanceA>=target_cm or distanceB>=target_cm) 
  {  //Arete les moteurs apres avoir fait la distance cible
    digitalWrite(MoteurA1, LOW);
    digitalWrite(MoteurA2, LOW);
    digitalWrite(MoteurB1, LOW);
    digitalWrite(MoteurB2, LOW);
    delay(2000);
    commande(target_cm,1,1);
    n++;
  }
}

//MOTEUR A
void doEncoderA1() {
  A1_set = digitalRead(encoderPinA1) == HIGH;
  encoder0PosA++;  //modifie le compteur selon les deux états des encodeurs
}
//---- Interruption appelée à tous les changements d'état de B

//MOTEUR B
void doEncoderB1() {
  B1_set = digitalRead(encoderPinB1) == HIGH;
  encoder0PosB++;  //modifie le compteur selon les deux états des encodeurs
}
//---- Fonction appelée pour contrôler le moteurA
void TournerA(int rapportCyclique, bool rotationMD_1,bool rotationMD_2) {
  analogWrite(pinPowerA, rapportCyclique);
  if (rapportCyclique > 0) 
  {
    digitalWrite(MoteurA1, rotationMD_2);
    digitalWrite(MoteurA2, rotationMD_1);
  } 
  else 
  {
    digitalWrite(MoteurA1, rotationMD_1);
    digitalWrite(MoteurA2, rotationMD_2);
    rapportCyclique = -rapportCyclique;
  }
}
//MOTEUR B
void TournerB(int rapportCyclique,bool rotationMG_1,bool rotationMG_2) {
  analogWrite(pinPowerB, rapportCyclique);
  if (rapportCyclique > 0)
   {
    digitalWrite(MoteurB1, rotationMG_2);
    digitalWrite(MoteurB2, rotationMG_1);
  } 
  else 
  {
    digitalWrite(MoteurB1, rotationMG_1);
    digitalWrite(MoteurB2, rotationMG_2);
    rapportCyclique = -rapportCyclique;
  }
}

void carre()
{   
  commande((target_cm/2),1,0);
}

void cercle()
{
  digitalWrite(MoteurA1,50);
  digitalWrite(MoteurA2,LOW);
  digitalWrite(MoteurB1,LOW);
  digitalWrite(MoteurB2,LOW);
}


void triangle()
{
  commande((target_cm*0.75),1,0); 
}

void commande(int distance,int sensrotationG,int sensrotationD )
{
  target_cm = distance;
  target_deg = 18 * target_cm;
  target_ticks = target_deg * (120.0 * 16.0) / 360.0;
  encoder0PosA = 0;  //position de départ=0
  boolean A1_set = false;
  encoder0PosB = 0;  //position de départ=0
  boolean B1_set = false;
  erreurA = 0;  //erreur
  erreurPrecedenteA = 0;
  somme_erreurA = 0;
  erreurB = 0;  //erreur
  erreurPrecedenteB = 0;
  somme_erreurB = 0;
  if(sensrotationG==1) 
  {
    rotationMG_1=1;
    rotationMG_2=0;
  }
  else if(sensrotationG==0)
  {
    rotationMG_1=0;
    rotationMG_2=1;
  }
  if(sensrotationD==1)
  {
    rotationMD_1=1;
    rotationMD_2=0;
  }
  if(sensrotationD==0)
  {
    rotationMD_1=0;
    rotationMD_2=1;
  }
}





