/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

/**
 * Add your docs here.
 */
public class TecbotMotorList {
    private List<TecbotSpeedController> motors;
    private HashMap<Integer, TecbotSpeedController> motorsHashMap;

    /**
     * Creates a new list of {@link TecbotSpeedController}.
     * With this class you can control a whole set of motors by just
     * calling one method.
     * It can also be used to access a motor in a specific port in that list.
     *
     * @param ports              Requires an array of ports
     * @param invertedMotorPorts Requires an array that indicates inverted motor ports
     * @param motorTypes      Requires an array of {@link TypeOfMotor}
     */
    public TecbotMotorList(int[] ports, int[] invertedMotorPorts, TypeOfMotor[] motorTypes) {
        motorsHashMap = new HashMap<>();
        motors = new ArrayList<>();
        for (int i = 0; i < ports.length; i++) {
            motors.add(new TecbotSpeedController(ports[i], motorTypes[i]));
            for (int port : invertedMotorPorts) {
                if (port == ports[i])
                    motors.get(i).setInverted(true);
            }
            motorsHashMap.put(ports[i], motors.get(i));
        }

    }

    /**
     * Sets all motors in the list to desired speed.
     *
     * @param speed power
     */
    public void setAll(double speed) {
        for (TecbotSpeedController motor : motors) {
            motor.set(speed);
        }
    }

    public void setSpecificMotor(int port, double speed){
        motorsHashMap.get(port).set(speed);
    }
}