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

    public static int[] LEFT_CHASSIS_PORTS = {2,3};
    public static int[] RIGHT_CHASSIS_PORTS = {0,1};

    public static TypeOfMotor[] LEFT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.PWM_TALON_SRX,TypeOfMotor.SPARK};
    public static TypeOfMotor[] RIGHT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.PWM_TALON_SRX,TypeOfMotor.PWM_TALON_SRX};

    /*
     * If encoder is connected to speed controller,
     * indicate the speed controller port here, and put encoder ports in
     * config not set
     */
    public static int LEFT_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;
    public static int RIGHT_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;
    public static int MIDDLE_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;

    /*
     * If encoder is connected to RoboRIO,
     * indicate the port here, and put encoder motor ports in
     * config not set
     */
    public static int[] LEFT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static int[] RIGHT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static int[] MIDDLE_WHEEL_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};

    /*
     * If any of the motors of the chassis must be inverted,
     * indicate the port(s) in these arrays.
     */
    public static int[] LEFT_CHASSIS_INVERTED_MOTORS = {};
    public static int[] RIGHT_CHASSIS_INVERTED_MOTORS = {};

    public static int TRANSMISSION_PORT[] = {0,1};

    public static int[] WHEEL_SOLENOID_PORTS = {2,3};
    public static int[] MIDDLE_WHEEL_PORTS ={};
    public static int[] MIDDLE_WHEEL_INVERTED_MOTORS ={};
    public static TypeOfMotor[] MIDDLE_WHEEL_MOTOR_TYPES = {};

    public static boolean DRAGON_FLY_IS_AVAILABLE = true;
}
