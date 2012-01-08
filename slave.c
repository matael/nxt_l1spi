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
  BT_WaitConn(MASTER);
  
  
  // connection etablished
  while(true){
    // wait & read the master's order
    byte *order = BT_ReadFromMaster();
    
    // call the right function
    if (order[0] == BOT_ROTATE_MOTOR_EX) {
        BT_RotateMotorEx(order[1], order[2], order[3], order[4], order[5]);
    } else if (order[0] == BOT_ON_FWD) {
        BT_OnFwd(order[1], order[2]);
    } else {
        __BT_OneByteFunc(order[1]);
    }
  } // end while
} // end task main()
