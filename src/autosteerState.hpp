#pragma once

#include <stdio.h>
#include "Arduino.h"

class SteerState {

  private:
    bool disengageState = true;
    bool softwareToHardwareMismatch = true;   // true when hardware engage signal changed, and software is not engaged yet
    bool autosteerSafetyLock = false;  // true when software sends engaged and hardware disengage is active
    bool enableAutosteerTimeout = false;  // true if software does not engage within 1 second of hardware engaging
    bool engagePrevState = false;
    bool disengagePrevState = false;
    bool softwareEngageState = false;  // software autosteer state
    bool hardwareEngageState = false;  // hardware autosteer state
    bool disabledByMaxEngageSpeed = false;
    time_t engageActivityMillis;
    time_t disengageActivityMillis;
    time_t softwareMessageMillis;

  public:
    double speed = 0.00;
    double maxEngageSpeed = 20.00;

  bool getSoftwareToHardwareMismatch(){
    return softwareToHardwareMismatch;
  }

  bool getEnableAutosteerTimeout(){
    return enableAutosteerTimeout;
  }

  bool getDisabledByMaxEngageSpeed(){
    return disabledByMaxEngageSpeed;
  }

  bool getAutosteerSafetyLock(){
    return autosteerSafetyLock;
  }

  bool getHardwareState(){
    return hardwareEngageState;
  }

  void fromHardware ( bool var ){
    if( hardwareEngageState != var ){
      engagePrevState = var;
      engageActivityMillis = millis();
    }
    if( var == true ){
      if( speed > maxEngageSpeed ){
        autosteerSafetyLock = true;
        disabledByMaxEngageSpeed = true;
      }
      else if( disengageState == true ){
        autosteerSafetyLock = true;
      }
      else {
        hardwareEngageState = true;
        softwareToHardwareMismatch = true;
        enableAutosteerTimeout = false;
      }
    }
    else {
      hardwareEngageState = false;
      softwareToHardwareMismatch = true;
      disabledByMaxEngageSpeed = false;
    }
  }

  bool toHardware(){
    if( autosteerSafetyLock == true ){
      hardwareEngageState = false;
    }
    else if( disengageState == true ){
      hardwareEngageState = false; // disable autosteer due to disengage
    }
    else if( millis() - engageActivityMillis > 1000 && softwareEngageState == false && hardwareEngageState == true ){
      hardwareEngageState = false; // disable autosteer due to timeout
      softwareToHardwareMismatch = true;
    }
    else if( millis() - softwareMessageMillis > 5000 ){
      hardwareEngageState = false; // disable autosteer due to timeout
      softwareToHardwareMismatch = true;
    }
    else if( softwareEngageState == true ){
      enableAutosteerTimeout = false;
      if( hardwareEngageState == true ){
        return true;
      }
    }
    return hardwareEngageState;
  }

  void setDisengage ( bool var ){
    if( disengagePrevState != var ){
      disengagePrevState = var;
      disengageActivityMillis = millis();
      softwareToHardwareMismatch = true;
    }
    if( var == true ){
      hardwareEngageState = false;
    }
    disengageState = var;
  }

  void fromSoftware( bool var ){
    // do not run function without message from software
    softwareMessageMillis = millis();
    if( softwareEngageState != var ){
      softwareEngageState = var;
      engageActivityMillis = millis();
      if( softwareEngageState == true ){
        if( speed > maxEngageSpeed || disengageState == true ){
          autosteerSafetyLock = true;
        }
      }
    }
    if( millis() - engageActivityMillis > 300  ){ // wait in case of network/software delay
      if( hardwareEngageState == softwareEngageState ){
        softwareToHardwareMismatch = false; // clear mismatch flag when autosteer control and AOG agree again
      }
      else if( softwareToHardwareMismatch == false ){
        if( autosteerSafetyLock == false ){
          // user pressed AOG autosteer button in software, we need to ACK so AOG listens to disengage
          hardwareEngageState = softwareEngageState; // reflect AOG state, will ACKNOWLEDGE when returned to AOG
        }
      }
    }
    if( softwareEngageState == false && hardwareEngageState == false ){
      autosteerSafetyLock = false;
    }
  }

  void printDebug(){
    Serial.print("safety ");
    Serial.print( autosteerSafetyLock );
    Serial.print(" hardware ");
    Serial.print( hardwareEngageState );
    Serial.print(" software ");
    Serial.print( softwareEngageState );
    Serial.print(" mismatch ");
    Serial.print( softwareToHardwareMismatch );
    Serial.print(" disengage ");
    Serial.println( disengageState );
  }

  bool toSoftware(){
    if( millis() - engageActivityMillis < 300 && autosteerSafetyLock == true && softwareEngageState == true ){
      return true; // send `on` for 300 millis then send actual state again afterwards
    } else {
      return hardwareEngageState;
    }
  }

};