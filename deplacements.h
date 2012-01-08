/*
* NXT to NXT bt communication
* Moves definitions
*
* Authors :
* Lucas Bourneuf
* Mathieu Gaborit
*
* Date : Janvier 2012
* License : WTFPL
*
*/



// faire un quart de tour par la droite
void quart_tour_d() {
  RotateMotorEx(motors, 90, 590, 100, true, false);
}   


// faire un quart de tour par la gauche
void quart_tour_g() { 
  RotateMotorEx(motors, 90, 590, -100, true, false);
}  


// deux quarts de tour, car lui faire faire un demi-tour avec RotateMotorEx() est imprécis
void demi_tour() { 
  quart_tour_g(); quart_tour_g(); 
} 


// virage à gauche
void Virage_gauche() { 
  left_speed = VITESSE_MIN_MOTEUR; right_speed = VITESSE_MAX_MOTEUR;
} 


// virage à droite
void Virage_droite() { 
  left_speed = VITESSE_MAX_MOTEUR; right_speed = VITESSE_MIN_MOTEUR; 
} 


//  décalage lors de la gestion d'obstacle
void Decalage() { 
  RotateMotor(motors, VITESSE_DE_CROISIERE, 550); 
} 



