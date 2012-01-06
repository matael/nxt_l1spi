/*
 * Protocole de communication bluetooth entre NXT.
 * Lucas Bourneuf
 * Mathieu Gaborit
 *
 * License : WTFPL
 */

#define MASTER 0
#define SLAVE 1
#define MAILBOX 0

// Commandes
#define BOT_ROTATE_MOTOR_EX 0xAE
#define BOT_ON_FWD 0xF0

// One Byte Commands
#define BOT_OFF 0x00
#define BOT_QUART_TOUR_D 0xC0
#define BOT_QUART_TOUR_G 0xC1
#define BOT_VIRAGE_D 0xB0
#define BOT_VIRAGE_G 0xB1
#define BOT_DEMI_TOUR 0xD1
#define BOT_DECALAGE 0xDE
// One Byte Func
#define BT_Off() __BT_OneByteFunc(BOT_OFF)
#define BT_QuartTourD() __BT_OneByteFunc(BOT_QUART_TOUR_D)
#define BT_QuartTourG() __BT_OneByteFunc(BOT_QUART_TOUR_G)
#define BT_VirageGauche() __BT_OneByteFunc(BOT_VIRAGE_G)
#define BT_VirageDroite() __BT_OneByteFunc(BOT_VIRAGE_D)
#define BT_DemiTour() __BT_OneByteFunc(BOT_DEMI_TOUR)
#define BT_Decalage() __BT_OneByteFunc(BOT_DECALAGE)



// Motors
#define BOT_SELECT_MOTOR_B 0xFB
#define BOT_SELECT_MOTOR_C 0xFC

// Réponses
#define SLAVE_ALL_IS_OK 0x42
#define SLAVE_NOTHING_TO_DO 0xD0
#define SLAVE_BUMPER 0xBB
#define SLAVE_C_LA_DECH 0xCD


// Attente d'une connexion
void BT_WaitConn(int conn);
// Verification de la connexion
void BT_CheckConn(int conn);


// ----- Variables globales -----
// __generic_slave_response_array est inutile car apparement, les données sont déjà enpaquetées
//byte __generic_slave_response_array[5] = {0xFF, 0x00, 0x42, 0x00, 0xFF};


void BT_WaitConn(int conn){
    state state; // état de la connexion
    do {
        state = BluetoothStatus(conn);

        if(state == NO_ERR) break;
        if(state == STAT_COMM_PENDING) continue;

        // on a une erreur, on avertir l'user
        TextOut(0, LCD_LINE2, "Erreur bluetooth", true);
        NumOut(30, LCD_LINE4, state);

        switch(state) {
            // Erreur de canal
            case ERR_COMM_CHAN_NOT_READY:
                TextOut(0, LCD_LINE6, "Pas de brique");
                TextOut(0, LCD_LINE7, "connecte en bluetooth !");
                break;

            // Erreur de bus
            case ERR_COMM_BUS_ERROR:
                TextOut(0, LCD_LINE6, "Probleme sur le bus");
                TextOut(0, LCD_LINE7, "Redemarrez !");
                break;
        }
        // on laisse le temps de lire
        Wait(7000);
        Stop(true); // on arrête le programme

    } while (state!=NO_ERR);
}


void BT_CheckConn(int conn){
    byte state = BluetoothStatus(conn);
    
    if (state == NO_ERR) {
        return;
    } else {
        // On a erreur 
        TextOut(0, LCD_LINE1, "Probleme Bluetooth", true);
        TextOut(0, LCD_LINE3, "Erreur : ");
        NumOut(30, LCD_LINE4, state);
        TextOut(0, LCD_LINE6, "Canal : ");
        NumOut(70, LCD_LINE6, conn);

        if (conn==SLAVE) {
            // nous sommes slave, en attente d'un master
            TextOut(0, LCD_LINE1, "En attente d'un");
            TextOut(20, LCD_LINE2, "maitre");
        } else {
            TextOut(0, LCD_LINE1, "En attente");
            TextOut(0, LCD_LINE2, "d'esclaves");
        }

        Wait(11000);
        Stop(true);
    }
}



void BT_SlaveResponseSend(byte response){
    // creation du buffer d'envoi
    byte _send_buffer[5];
    // les 4 octets suivants semblent obligatoires
    _send_buffer[0] = 0x80;       // "no reply telegram"
    _send_buffer[1] = 0x09;       // "MessageWrite Direct Command"
    _send_buffer[2] = MAILBOX;    // selection de la boite aux lettres
    _send_buffer[3] = 1;          // Longueur de la donnée

    // On ajoute la donnée
    _send_buffer[4] = reponse;

    // on envoie le tout
    BT_WaitConn(MASTER);
    BluetoothWrite(MASTER,_send_buffer);
    BT_WaitConn(MASTER);
}



void __BT_MasterCommandSend(byte *command_array, int array_len){
    // Création du buffer d'envoi
    byte* _send_buffer = malloc((4+array_len)*sizeof(byte));
    _send_buffer[0] = 0x80;       // "no reply telegram"
    _send_buffer[1] = 0x09;       // "MessageWrite Direct Command"
    _send_buffer[2] = MAILBOX;    // selection de la boite aux lettres
    _send_buffer[3] = array_len;          // Longueur de la donnée

    // ajout de la donnée
    int i = 4;
    for (i = 0; i < array_len+4; i++) {
        _send_buffer[i] = command_array[i-4];
    }

    // envoi
    BT_WaitConn(SLAVE);
    BluetoothWrite(SLAVE, _send_buffer);
    BT_WaitConn(SLAVE);

    // liberation du buffer
    free(_send_buffer);
}

void __BT_OneByteFunc(byte commande){
    byte _command_buffer[1] = {commande};
    __BT_MasterCommandSend(_command_buffer, 1);
}


void BT_OnFwd(byte motor, byte pwr){
    // partie "commande de la trame"
    byte _command_buffer[2] = {motor,pwr};
    // envoi
    __BT_MasterCommandSend(_command_buffer, 2);
}

void BT_RotateMotorEx(byte power, byte angle, byte turn_ratio, bool sync_bool, bool stop_bool){
    // Force la conversion booléen -> octet
    byte sync = (sync_bool) ? 0x01 : 0x00;
    byte stop = (stop_bool) ? 0x01 : 0x00;
    // on note le tout dans une variable pour une simple raison de __lisibilité_
    byte _command_buffer[6] = {BOT_ROTATE_MOTOR_EX, power, angle, turn_ratio, sync, stop};
    __BT_MasterCommandSend(_command_buffer, 6);
}