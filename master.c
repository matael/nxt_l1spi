/*
 * NXT to NXT bt communication
 * Master brick.
 *
 * Authors :
 *  Lucas Bourneuf
 *  Mathieu Gaborit
 *
 * Date    : Janvier 2012
 * License : WTFPL
 *
 */

#include "protocol.h"



//======================
//       DEFINES
//======================
// maitre
#define BOT_OFF 0x00
#define BOT_ROTATE_MOTOR_EX 0xAE
#define BOT_ON_FWD 0xF0
#define BOT_SELEC_MOTOR_B 0xFB // moteur droit
#define BOT_SELEC_MOTOR_G 0xFC // moteur gauche

// appel des shorts fonctions
#define BOT_QUART_TOUR_D 0xC0
#define BOT_QUART_TOUR_G 0xC1
#define BOT_VIRAGE_D 0xB0
#define BOT_VIRAGE_D 0xB1
#define BOT_DEMI_TOUR 0xD1
#define BOT_DECALAGE 0xDE

// slave
#define SLAVE_ALL_IS_OK 0x42
#define SLAVE_NOTHING_TO_DO 0xD0
#define SLAVE_BUMPER 0xBB
#define SLAVE_C_LA_DECH 0xCD

// capteurs et moteurs
#define Couleur IN_2 // capteur couleur (vers le sol)
#define TVD IN_1 // télémètre dirigé vers la droite (sert à déterminer la fin d'un obstacle que l'on esquive)
#define TVA IN_3  // télémètre dirigé vers l'avant (sert à locialiser les obstacles)
#define Touch IN_4   // capteur tactile (arrêt d'urgence)
#define motors OUT_BC        // moteurs branchés sur les ports B et C

// télémètres et couleur
#define trop_a_droite (couleur_vue == INPUT_REDCOLOR)    // condition prédéfinie  (allège considérablement le code)
#define trop_a_gauche (couleur_vue == INPUT_GREENCOLOR)  // idem
#define DISTANCE_CHEMIN_LIBRE 20                         // distance à laquelle on considère que le chemin est libre
#define YA_UNE_LIGNE (couleur_vue == INPUT_BLACKCOLOR || couleur_vue == INPUT_REDCOLOR || couleur_vue == INPUT_GREENCOLOR) // vrai si on est sur une ligne (noir, rouge ou vert)

// programme
#define NB_MARQUE_MAX 18     // nombre de marques bleues nécessaire pour finir le circuit
#define VITESSE_DE_CROISIERE 65    // puissance des moteurs lors de parcours de piste
#define VITESSE_MAX_MOTEUR 80 // vitesse du moteur extérieur lors d'un virage
#define VITESSE_MIN_MOTEUR 20 // vitesse du moteur intérieur lors d'un virage

// sécurité
#define TEMPS_EMERGENCY_JAUNE 50   // temporisation après lecture de jaune, pour éviter des erreurs de lectures intempestives

// contantes utilisées pour l'état d'esprit de la machine   (enumérations non utilisables...)
#define SUIVRE_LIGNE 0
#define CHERCHE_LIGNE 1
#define ESQUIVE_OBSTACLE 2




//======================
//  VARIABLES GLOBALES
//======================
int couleur_vue;
int derniere_couleur_vue = INPUT_BLACKCOLOR;
int compteur = 0; // compteur des marques
int Etat_Esprit = CHERCHE_LIGNE; // définit ce que fait le robot (utilisation des defines)   (initialisé à cherche_ligne)
bool Robot_Fonctionne = true;  // le main ne peut fonctionner que si il est vrai
unsigned long first_tick = 0;  // utilisé pour sauvegarder le moment de démarrage du programme.
int left_speed = VITESSE_DE_CROISIERE;     // vitesse du moteur gauche
int right_speed = VITESSE_DE_CROISIERE;    // vitesse du moteur droit




//======================
//      PROTOTYPES
//======================
void setup(); // Initialisation et lancement du programme
task Emergency_stop();  // Tache gérant la fin de programme selon trois conditions non exclusives : bord de circuit, NB_MARQUE_MAX marques bleues ou bouton d'arrêt d'urgence
void affichage_FinDeProgramme(); // fait les affichages de fin de programme : le temps de parcours du robot pendant cinq secondes
bool Presence_Obstacle_Droite(); // fonction renvoyant vrai tant que le télémètre dirigé vers la droite (TV_d) trouve un objet à moins de 20 cm.
bool Presence_Obstacle_Avant();  // Retourne vrai si un obstacle est détecté à moins de 5 cm du télémètre avant. (TVA)
void Gestion_obstacle(); // gère l'esquive d'obstacle
void lecture_couleur_bleu(); // opérations à faire en cas de lecture de bleu
void Zonage_espace_immediat(); // recherche préliminaire de lignes en zonant dans l'espace immédiat
void recherche_de_ligne(); // recherche une ligne, puis, une fois trouvée, amorce son suivi dans le bon sens
void Amorcage_suivi_ligne(); // recherche une ligne, puis, une fois trouvée, amorce son suivi dans le bon sens





// Définitions des fonctions

// fonctions déclenchant le mouvement du maître et le lancement de l'ordre vers les esclaves associés
void MT_OnFwd(int motors, int speed){
    OnFwd(motors, speed);    // bouger le robot maître
    BT_OnFwd(motors, speed); // lancement de l'ordre à l'esclave
}

void MT_quart_tour_d(){
    MT_quart_tour_d();    // bouger le robot maître
    BT_QuartTourD(); // lancement de l'ordre à l'esclave
}

void MT_quart_tour_g(){
    MT_quart_tour_g();    // bouger le robot maître
    BT_QuartTourG(); // lancement de l'ordre à l'esclave
}

void MT_OnDecalage(){
    MT_OnDecalage();    // bouger le robot maître
    BT_Decalage(); // lancement de l'ordre à l'esclave
}

void MT_demi_tour(){
    MT_quart_tour_g(); // bouger le robot maître
    MT_quart_tour_g(); // lancement de l'ordre à l'esclave
}

void MT_Virage_gauche(){
    MT_Virage_gauche();    // bouger le robot maître
    BT_VirageGauche(); // lancement de l'ordre à l'esclave
}

void MT_Virage_droite(){
    MT_Virage_droite();    // bouger le robot maître
    BT_VirageDroite(); // lancement de l'ordre à l'esclave
}

void MT_RotateMotorEx(int motors, int speed, int angle, int ratio, int bol1, int bol2){
    OnFwd(motors, speed, angle, ratio, bol1, bol2);    // bouger le robot maître
    BT_OnFwd(motors, speed, angle, ratio, bol1, bol2); // lancement de l'ordre à l'esclave
}

void MT_Off(int motors){
    Off(motors); // bouger le robot maître
    BT_Off();    // lancement de l'ordre à l'esclave
}



//======================
//    SETUP
//======================
void setup() { // Initialisation et lancement du programme
     SetSensorLowspeed(TVD);      // port du télémètre orienté à droite        (étudier la possibilité de revenir sur un obstacle)
     SetSensorColorFull(Couleur);  // port couleur                              (voir la piste)
     SetSensorLowspeed(TVA);       // port télémètre orienté en avant           (voir les obstacles avant de s'écraser dessus)
     SetSensorTouch(Touch);        // port tactile                              (arrêt d'urgence)
     StartTask(Emergency_stop);    // tache de gestion de fin de prgm           (couleur jaune, compteur de marques bleues, arrêt d'urgence)
     first_tick = FirstTick();     // variable enregistrant le moment de départ du programme.
} // end setup()




//======================
//    EMERGENCY_STOP
//======================
 // Tache gérant la fin de programme selon trois conditions non exclusives : bord de circuit, NB_MARQUE_MAX marques bleues ou bouton d'arrêt d'urgence
task Emergency_stop(){
     while(Robot_Fonctionne)    // tant que le programme est considéré en fonctionnement
     {
          // si le capteur couleur voit du jaune, s'il à compté 18 marques ou plus, ou si le bouton est pressé
          if((couleur_vue == INPUT_YELLOWCOLOR)){
               Wait(TEMPS_EMERGENCY_JAUNE);  // protection pour éviter de s'arrêter si le capteur détecte du jaune par accident
               if(Sensor(Couleur) == INPUT_YELLOWCOLOR){
                       Robot_Fonctionne = false; // le main arrête de tourner.
                       MT_Off(motors);    // arrêt des moteurs
                       StopAllTasks(); // arrêt du programme
               }
          }
          else if(compteur >= NB_MARQUE_MAX || Sensor(Touch)) { // si on a égalé ou dépassé le nombre de marque bleues maximum, on s'arrête. De même si on enclenche le capteur tactile.
                Robot_Fonctionne = false; // le main arrête de tourner.
                MT_Off(motors);    // arrêt des moteurs
                affichage_FinDeProgramme(); // affichage du temps de parcours
                StopAllTasks(); // arrêt du programme
          }
     }
}// end Emergency_stop()




//======================
//    FIN DE PROGRAMME
//======================
// fait les affichages de fin de programme : le temps de parcours du robot pendant cinq secondes
void affichage_FinDeProgramme(){
    TextOut(10, LCD_LINE2, "Temps de");
    TextOut(20, LCD_LINE3, "parcours : ");
    TextOut(30, LCD_LINE4, NumToStr((CurrentTick() - first_tick)/1000)); // on affiche (moment DémarragePrgm - MomentActuel) (c'est égal au temps d'exécution de prgm)
    TextOut(10, LCD_LINE6, "Marques");
    TextOut(20, LCD_LINE7, "comptées : ");
    TextOut(30, LCD_LINE8, NumToStr(compteur));
    Wait(5000);     // attendre cinq secondes, qu'on puisse voir les résultats.
}// end affichage_FinDeProgramme()




//======================
//    TELEMETRES
//======================
// fonction renvoyant vrai tant que le télémètre dirigé vers la droite trouve un objet à moins de 20 cm.
bool Presence_Obstacle_Droite(){
     if(SensorUS(TV_d) <= DISTANCE_CHEMIN_LIBRE)
         return true;
     return false;
}

// retourne vrai si le télémètre de devant trouve un obstacle à moins de 5 centimètres.
bool Presence_Obstacle_Avant(){
     if(SensorUS(TVA) <= 5)
         return true;
     return false;
}




//======================
//    GESTION_OBSTACLE
//======================
// gère l'esquive d'obstacle
void Gestion_obstacle(){
     MT_RotateMotorEx(motors, VITESSE_DE_CROISIERE, -180, 0, true, false);
     MT_quart_tour_g();
     MT_OnFwd(motors, 50);
     while(Presence_Obstacle_Droite()){
          couleur_vue = Sensor(Couleur);
          if(YA_UNE_LIGNE){
               Etat_Esprit = CHERCHE_LIGNE;  // si on trouve une ligne, on amorçe la routine d'alignement avec ladite ligne
               return; // on quitte la fonction, car on ne considère plus d'obstacle
          }
     } // end boucle while
     MT_Decalage(); // on avance un peu pour que l'ensemble du robot ne soit pas en face de l'obstacle
     MT_quart_tour_d();

     MT_Decalage(); // on avance un peu pour que l'ensemble du robot ne soit pas en face de l'obstacle
     MT_OnFwd(motors, 50); // on avance...
     while(Presence_Obstacle_Droite()){ // ... tant qu'il y a un obstacle à droite
          couleur_vue = Sensor(Couleur);
          if(YA_UNE_LIGNE) { // si on rencontre une ligne, on quitte la fonction et on amorce la routine d'alignement avec ladite ligne
               Etat_Esprit = CHERCHE_LIGNE;
               return;
          }
     } // end boucle while
     MT_Decalage(); // on s'écarte de l'obstacle
     MT_quart_tour_d(); // on est théoriquement face à la ligne que l'obstacle à bouché !
     Etat_Esprit = CHERCHE_LIGNE; // on cherche cette ligne
}




//======================
//    LECTURE_BLEU
//======================
// opérations à faire en cas de lecture de bleu
void lecture_couleur_bleu(){
     if (couleur_vue != derniere_couleur_vue && Etat_Esprit == SUIVRE_LIGNE){
         Wait(5);
         if(Sensor(Couleur) == couleur_vue){
             compteur++;
             TextOut(25,LCD_LINE2, NumToStr(compteur));
         }
     }
     MT_OnFwd(motors,70);
}




//======================
//    ZONAGE D'ESPACE
//======================
// recherche préliminaire de lignes en zonant dans l'espace immédiat
void Zonage_espace_immediat(){
     MT_RotateMotorEx(motors, 100, 60, 100, true, true); // tourner sur quelques centimètres à droite
     couleur_vue = Sensor(Couleur);
     if(YA_UNE_LIGNE) {
         Etat_Esprit = SUIVRE_LIGNE;
         return;
     }
     MT_RotateMotorEx(motors, 100, 120, -100, true, true); // tourner sur quelques centimètres à gauche
     couleur_vue = Sensor(Couleur);
     if(YA_UNE_LIGNE) {
         Etat_Esprit = SUIVRE_LIGNE;
         return;
     }
     MT_RotateMotorEx(motors, 100, 60, 100, true, true); // tourne pour revenir dans l'axe
     couleur_vue = Sensor(Couleur);
     if(YA_UNE_LIGNE) {
         Etat_Esprit = SUIVRE_LIGNE;
         return;
     }
}




//======================
//    RECHERCHE_LIGNE
//======================
// recherche une ligne, puis, une fois trouvée, amorce son suivi dans le bon sens
void recherche_de_ligne(){
   Etat_Esprit = CHERCHE_LIGNE;
   Wait(10);  // protections
   if(Sensor(Couleur) == INPUT_WHITECOLOR){
         // recherche préliminaire de ligne dans l'espace immédiat
         Zonage_espace_immediat();

         // on avance jusqu'à trouver une ligne
         MT_OnFwd(motors, 100);
         while(!YA_UNE_LIGNE) // tant qu'il y a pas de lignes
             { couleur_vue = Sensor(Couleur); }

         // une ligne est trouvée !
         Amorcage_suivi_ligne(); // on amorce le suivis de cette ligne

   } // end if (Sensor(Couleur) == INPUT_WHITECOLOR)
   Etat_Esprit = SUIVRE_LIGNE; // on a trouvé une ligne !
   MT_OnFwd(motors,VITESSE_DE_CROISIERE);
}




//======================
//    AMORCAGE_SUIVI_LIGNE
//======================
// Amorce le suivis de ligne en s'aligant avec la ligne trouvée
void Amorcage_suivi_ligne(){
    MT_RotateMotorEx(motors, 100, 140, 0, false, true); // parcourir quelques centimètres pour dépasser la ligne
    while(couleur_vue != INPUT_BLACKCOLOR && couleur_vue != INPUT_BLUECOLOR) // si ya pas de ligne noire (ou bleue)
    {
        couleur_vue = Sensor(Couleur); // on regarde la couleur
        if(couleur_vue == INPUT_WHITECOLOR || couleur_vue == INPUT_REDCOLOR) // si blanc ou rouge, on tourne
            MT_RotateMotorEx(motors, 75, 10, -100, true, false); // vers la gauche, pour trouver un eligne noire dans le bon sens

        if(couleur_vue == INPUT_GREENCOLOR){ // si vert
            MT_RotateMotorEx(motors, 100, 60, 100, true, true); // on se tourne vers la droite
            couleur_vue = Sensor(Couleur); // on regarde la couleur
            if(YA_UNE_LIGNE){ // si ya une ligne
                  Etat_Esprit = SUIVRE_LIGNE; // on a trouvé une ligne !
                  MT_OnFwd(motors,VITESSE_DE_CROISIERE);  // on continue l'exploration
                  return;
            }
            else  // si c'est du blanc, c'est qu'on est pas dans le bon sens... donc on se retourne !
                  MT_demi_tour(); // on fait un demi tour pour dépasser la ligne sans passer par la ligne jaune
        } // end if (couleur_vue == INPUT_GREENCOLOR)
    } // end boucle d'attante de ligne noire ou bleu.
}




//======================
//    TASK MAIN
//======================
task main() { // main(), l'épine dorsale du prgm
     setup();   // intialisation
     recherche_de_ligne(); // première chose : on cherche une ligne

     // boucle principale de programme. Etudie les différentes couleurs
     while(Robot_Fonctionne)    // tant que le programme est considéré en fonctionnement
     {
          couleur_vue = Sensor(Couleur);   // Pour éviter de tester en permanence le capteur couleur, on utilisera une variable actualisée à chaque tour de boucle.

          // on teste la présence d'un obstacle avant la gestion des couleurs
          if(Presence_Obstacle_Avant()) {  // si il y a un obstacle devant
              Gestion_obstacle();         // on gère l'obstacle
              continue; // et on passe à la boucle suivante
          }

          // le robot marche, et pas d'obstacle : on gère les moteurs en fonction des speeds propres
          MT_OnFwd(OUT_C, left_speed);
          MT_OnFwd(OUT_B, right_speed);

          // switch maître sur la variable couleur_vue
          switch (couleur_vue)
          {
              case INPUT_REDCOLOR:        // ROUGE
                   MT_Virage_gauche(); // virage à gauche
                   break;

              case INPUT_GREENCOLOR:      // VERT
                   MT_Virage_droite(); // virage à droite
                   break;

              case INPUT_BLUECOLOR:      // BLEU
                   lecture_couleur_bleu(); // on a lu du bleu
                   break;

              case INPUT_YELLOWCOLOR:    // JAUNE
                  Wait(TEMPS_EMERGENCY_JAUNE);  // protection pour éviter de s'arrêter si le capteur détecte du jaune par accident
                  if(Sensor(Couleur) == INPUT_YELLOWCOLOR)
                        MT_Off(motors); // extinction des moteurs. La tache emergency_stop fera le reste si c'est pas déjà fait
                  break;

              case INPUT_WHITECOLOR:     // BLANC
                  recherche_de_ligne();
                  break;


              default:  // toute autre cas, dont la lecture de noir
                  if(Robot_Fonctionne) // si le robot fonctionne toujours
                      MT_OnFwd(motors,VITESSE_DE_CROISIERE);
          } // end switch

          // actualisation de la dernière couleur vue
          derniere_couleur_vue = couleur_vue;
     } // end boucle principale
} // end main()






// fin du prgm



