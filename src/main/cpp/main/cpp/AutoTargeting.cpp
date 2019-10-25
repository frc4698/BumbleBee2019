/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <main/include/AutoTargeting.h>

double AutoTargeting::AutoTargetTurn(){       
        tx = table->GetNumber("tx",0.0);
        float steering_adjust = 0.0f;
        if (tx > 1.0)
        {
                steering_adjust = tP * tx - kF;
        }
        else if (tx < -1.0)
        {
                steering_adjust = (tP * tx + kF);
        }
        return(steering_adjust);
}
