//*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;


public class RobotMap {

    public static int[] leftChassisPorts = {0,1,2};
    public static int[] rightChassisPorts = {0,1,2};

    public static TypeOfMotor[] leftChassisMotorTypes = {TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX ,TypeOfMotor.TALON_SRX};
    public static TypeOfMotor[] rightChassisMotorTypes = {TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX ,TypeOfMotor.TALON_SRX};

    /*
     * If encoder is connected to speed controller,
     * indicate the speed controller port here, and put encoder ports in
     * config not set
     */
    public static int leftChassisMotorWithEncoder = RobotConfigurator.CONFIG_NOT_SET;
    public static int rightChassisMotorWithEncoder = RobotConfigurator.CONFIG_NOT_SET;

    /*
     * If encoder is connected to RoboRIO,
     * indicate the port here, and put encoder motor ports in
     * config not set
     */
    public static int[] leftChassisEncoderPorts = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static int[] rightChassisEncoderPorts = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static int[] middleWheelEncoderPorts = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};

    /*
     * If any of the motors of the chassis must be inverted,
     * indicate the port(s) in these arrays.
     */
    public static int[] leftChassisInvertedMotors = {};
    public static int[] rightChassisInvertedMotors = {};

    public static int transmissionPorts[] = {0,1};

    public static int[] wheelSolenoidPorts = {0,1};
    public static int middleWheelPort = 0;
    public static TypeOfMotor middleWheelMotorType = TypeOfMotor.TALON_SRX;

    public static final int[]  SHARED_ENCODER_PORT = {0,0};
	public static final int[] LEFT_CHASSIS_ENCODER_PORTS = {0,0};
	public static final int[] RIGHT_CHASSIS_ENCODER_PORTS = null;
	public static final int[] MIDDLE_WHEEL_ENCODER_PORTS = null;

    public static final  int[] SHARED_MOTORS_LEFT_PORTS = {7, 8};
    public static final int[] SHARED_MOTORS_RIGHT_PORTS = {9, 10};
    public static final int[] INVERTED_SHARED_MOTORS_LEFT = {0, 0};
    public static final int[] INVERTED_SHARED_MOTORS_RIGHT = {0, 0};
    public static final int SHARED_RIGHT_MOTOR_WITH_ENCODER = 0;
    public static final int SHARED_LEFT_MOTOR_WITH_ENCODER = 0;
    public static TecbotSpeedController.TypeOfMotor[] SHARED_RIGHT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};
    public static TecbotSpeedController.TypeOfMotor[] SHARED_LEFT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};

    public static final int ANGLER_PORT = 0;
    public static double MANUALSHOOT = 0.4;
  

}
