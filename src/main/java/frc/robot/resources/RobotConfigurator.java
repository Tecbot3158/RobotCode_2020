/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

/**
 * Add your docs here.
 */
public class RobotConfigurator {

    public static int CONFIG_NOT_SET = -1;

    public static TecbotEncoder buildEncoder(TecbotSpeedController speedController, int a, int b){
        if( speedController != null){

            if( speedController.getType() == TypeOfMotor.TALON_SRX  )
                return new TecbotEncoder(speedController);

        }
        if( a != CONFIG_NOT_SET && b > CONFIG_NOT_SET && a != b)
            return new TecbotEncoder(a,b);
        
        DriverStation.reportWarning("No ENCODER on this build try", true);
        return null;
    }

    /**
     * Builds a new {@link TecbotMotorList}
     *
     * @param motorPorts
     * @param invertedPorts
     * @param typesOfMotors
     * @return A new {@link TecbotMotorList}
     */
    public static TecbotMotorList buildMotorList(int[] motorPorts, int[] invertedPorts, TypeOfMotor[] typesOfMotors) {
        return new TecbotMotorList(motorPorts, invertedPorts, typesOfMotors);
    }
}


