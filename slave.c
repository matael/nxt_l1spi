/*
 * NXT to NXT bt communication
 * Slave brick.
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

task main(){
  // wait for a connection to the master
  BT_ChackConn(SLAVE);
  
  
  // connection etablished
  while(true){
    // wait & read the master's order
    byte *order = BT_ReadFromMaster();
    
    // call the right function
    if (order[0] == BOT_ROTATE_MOTOR_EX) {
        Slave_RotateMotorEx(order);
    } else if (order[0] == BOT_ON_FWD) {
        Slave_OnFwd(order);
    } else {
        Slave_OneByteDecode(order[1]);
    }
  } // end while
} // end task main()
