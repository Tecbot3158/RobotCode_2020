/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

public class RobotMap {

    public static int[] leftChassisPorts = { 2, 3 };
    public static int[] rightChassisPorts = { 0, 1 };

    public static TypeOfMotor[] leftChassisMotorTypes = { TypeOfMotor.PWM_TALON_SRX, TypeOfMotor.SPARK };
    public static TypeOfMotor[] rightChassisMotorTypes = { TypeOfMotor.PWM_TALON_SRX, TypeOfMotor.PWM_TALON_SRX };

    /*
     * If encoder is connected to speed controller, indicate the speed controller
     * port here, and put encoder ports in config not set
     */
    public static int leftChassisMotorWithEncoder = RobotConfigurator.CONFIG_NOT_SET;
    public static int rightChassisMotorWithEncoder = RobotConfigurator.CONFIG_NOT_SET;

    /*
     * If encoder is connected to RoboRIO, indicate the port here, and put encoder
     * motor ports in config not set
     */
    public static int[] leftChassisEncoderPorts = { RobotConfigurator.CONFIG_NOT_SET,
            RobotConfigurator.CONFIG_NOT_SET };
    public static int[] rightChassisEncoderPorts = { RobotConfigurator.CONFIG_NOT_SET,
            RobotConfigurator.CONFIG_NOT_SET };
    public static int[] middleWheelEncoderPorts = { RobotConfigurator.CONFIG_NOT_SET,
            RobotConfigurator.CONFIG_NOT_SET };

    /*
     * If any of the motors of the chassis must be inverted, indicate the port(s) in
     * these arrays.
     */
    public static int[] leftChassisInvertedMotors = {};
    public static int[] rightChassisInvertedMotors = {};

    public static int transmissionPorts[] = { 0, 1 };

    public static int[] wheelSolenoidPorts = { 2, 3 };
    public static int middleWheelPort = 0;
    public static TypeOfMotor middleWheelMotorType = TypeOfMotor.TALON_SRX;

    /*
     * Front Intake Subsystem
     */

    public static int[] backIntakeMotors = { 0 };
    public static int[] frontIntakeMotors = { 1 };

    public static boolean[] frontIntakeMotorDirection = { true };
    public static boolean[] backIntakeMotorDirection = { true };

    public static TypeOfMotor[] frontIntakeMotorsTypes = { TypeOfMotor.TALON_SRX };
    public static TypeOfMotor[] backIntakeMotorsTypes = { TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX };

    public static int[] frontsolenoids = { 0, 1 };
    public static int[] backsolenoids = { 2, 3 };

    /* Test */
}
