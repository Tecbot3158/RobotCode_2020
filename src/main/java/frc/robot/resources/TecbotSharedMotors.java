/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import java.util.List;

import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

/**
 * Add your docs here.
 */
public class TecbotSharedMotors {
    public static List<TecbotSpeedController> rightSharedMotors, leftSharedMotors;
    private int encoderMotor = RobotMap.shooterClimberMotorWithEncoder;

    public static void initializeSharedMotors() {
        for (int i = 0; i < RobotMap.shooterClimberPortsRight.length; i++) {
            rightSharedMotors.add(new TecbotSpeedController(RobotMap.shooterClimberPortsRight[i], RobotMap.SHOOTERCLIMBER_TYPE_OF_MOTORS[0]));
            if (RobotMap.shooterClimberPortsRight[i] == RobotMap.invertedLeftShooterClimberMotors[i]) {
                rightSharedMotors.get(i).setInverted(true);
            }
        }
        for (int i = 0; i < RobotMap.shooterClimberPortsLeft.length; i++) {
            leftSharedMotors.add(new TecbotSpeedController(RobotMap.shooterClimberPortsLeft[i], RobotMap.SHOOTERCLIMBER_TYPE_OF_MOTORS[0]));
            if (RobotMap.shooterClimberPortsLeft[i] == RobotMap.invertedRightShooterClimberMotors[i]) {
                leftSharedMotors.get(i).setInverted(true);
            }
        }
    }

    /**
     * @param speedRight speed for right motors
     * @param speedLeft  speed for left motors
     */
    public static void setAll(double speedRight, double speedLeft) {
        for (TecbotSpeedController motor : rightSharedMotors) {
            motor.set(speedRight);
        }
        for (TecbotSpeedController motor : leftSharedMotors) {
            motor.set(speedLeft);
        }
    }

    public TecbotSpeedController getMotorWithEncoderRight() {
        return (encoderMotor > 0) ? rightSharedMotors.get(encoderMotor) : null;
    }

    public TecbotSpeedController getMotorWithEncoderLeft() {
        return (encoderMotor > 0) ? leftSharedMotors.get(encoderMotor) : null;
    }
}
