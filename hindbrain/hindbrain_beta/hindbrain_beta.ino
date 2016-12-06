/* 
 * hindbrain_beta
 * A quick hindbrain with a ROSSerial pipe. Oy vey!
 */

#include "includes.h"
#include "defines.h"
#include "pins.h"
#include "global.h"

void setup()
{ 
  //Initialize Hardware
  setupMotors();
  setupROS();
}

void loop()
{  
  publishAtInterval();
  checkIR();
  updateNeopixels();

  switch (mode) {
    case OK_MODE:
      runMotors();
      break;
    case PHYS_ESTOP_MODE:
      stopMotors();
      break;
    case IR_DETECTED_MODE:
      stopMotors();
      break;
    default:
      mode = PHYS_ESTOP_MODE;
      break;
  }

  //ROS
  nh.spinOnce();
  
}


