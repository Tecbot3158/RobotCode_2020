//*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.resources.RobotConfigurator;
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

<<<<<<< HEAD
    public static final TypeOfMotor [] SHOOTERCLIMBER_TYPE_OF_MOTORS= {TypeOfMotor.TALON_SRX};
    
    public static int[] shooterClimberPortsLeft = {7, 8};
    public static int[] shooterClimberPortsRight = {9, 10};
    public static int[] invertedLeftShooterClimberMotors = {0, 0};
    public static int[] invertedRightShooterClimberMotors = {0, 0};
    public static int shooterClimberMotorWithEncoder = 0;
=======
    public static final TypeOfMotor [] SHOOTER_TYPE_OF_MOTORS= {TypeOfMotor.TALON_SRX,TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX };
    public static final int [] SHOOTER_LEFT_MOTOR_PORTS = {7,8};
    public static final int [] SHOOTER_RIGHT_MOTOR_PORTS = {9,10};
    public static final int [] SHOOTERENCODER_PORT = {0,1};

    public static final int [] SHOOTER_LEFT_INVERTED_MOTOR_PORTS = {};
    public static final int [] SHOOTER_RIGHT_INVERTED_MOTOR_PORTS = {};

>>>>>>> 5f53229b591949e547c60be388d5ea562cfee634
    public static final int ANGLERPORT = 0;

    

}
