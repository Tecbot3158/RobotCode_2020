/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import java.util.ArrayList;
import java.util.List;

import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

public class TecbotMotorList {
    private List<TecbotSpeedController> motors;

    /**
     * Creates a new <strong>list</strong> of {@link TecbotSpeedController}s
     *
     * @param motorPorts    Requires an array of ports
     * @param invertedPorts Requires an array that indicates inverted motor ports
     * @param typesOfMotors Requires an array of {@link TypeOfMotor}s
     */
    public TecbotMotorList(int[] motorPorts, int[] invertedPorts, TypeOfMotor[] typesOfMotors) {
        motors = new ArrayList<>();
        for (int i = 0; i < motorPorts.length; i++) {
            motors.add(new TecbotSpeedController(invertedPorts[i], typesOfMotors[i]));
            for (int port : invertedPorts) {
                if (port == motorPorts[i])
                    motors.get(i).setInverted(true);
            }
        }
    }

    /**
     * Sets all motors in the list to a speed
     * @param speed power
     */
    public void setAll(double speed) {
        for (TecbotSpeedController motor : motors) {
            motor.set(speed);
        }
    }

    public TecbotSpeedController getMotor(int index){
        return motors.get(index);
    }
}