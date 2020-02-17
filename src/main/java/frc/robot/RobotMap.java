//*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

public class RobotMap {


    public static int[] LEFT_CHASSIS_PORTS = {17, 5};
    public static int[] RIGHT_CHASSIS_PORTS = {1,3};

    public static TypeOfMotor[] LEFT_CHASSIS_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};
    public static TypeOfMotor[] RIGHT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};


    /*
     * If encoder is connected to speed controller,
     * indicate the speed controller port here, and put encoder ports in
     * config not set
     */
    public static int LEFT_CHASSIS_MOTOR_WITH_ENCODER = 5;
    public static int RIGHT_CHASSIS_MOTOR_WITH_ENCODER = 1;
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
    public static int[] LEFT_CHASSIS_INVERTED_MOTORS = {17,5};
    public static int[] RIGHT_CHASSIS_INVERTED_MOTORS = {};

    public static boolean LEFT_CHASSIS_ENCODER_IS_INVERTED = true;
    public static boolean RIGHT_CHASSIS_ENCODER_IS_INVERTED = false;
    public static boolean MIDDLE_CHASSIS_ENCODER_IS_INVERTED = false;

    public static int[] TRANSMISSION_SOLENOID_PORTS = {1,2, 3};
    public static final DoubleSolenoid.Value TORQUE_TRANSMISSION = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value SPEED_TRANSMISSION = DoubleSolenoid.Value.kReverse;

    public static int[] MIDDLE_WHEEL_PORTS = {2};
    public static int[] MIDDLE_WHEEL_INVERTED_MOTORS = {};
    public static TypeOfMotor[] MIDDLE_WHEEL_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS};

    public static int[] WHEEL_SOLENOID_PORTS = {1, 0, 1};
    public static final DoubleSolenoid.Value LOWERED_WHEEL = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value RAISED_WHEEL = DoubleSolenoid.Value.kReverse;
    public static boolean DRAGON_FLY_IS_AVAILABLE = true;

    /*
     * SHARED MOTORS STARTS
     */
    public static int[] SHARED_MOTORS_LEFT_PORTS = {7, 8};
    public static int[] SHARED_MOTORS_RIGHT_PORTS = {9, 10};
    public static int[] INVERTED_SHARED_MOTORS_LEFT = {0, 0};
    public static int[] INVERTED_SHARED_MOTORS_RIGHT = {0, 0};
    public static int SHARED_RIGHT_MOTOR_WITH_ENCODER = 0;
    public static int SHARED_LEFT_MOTOR_WITH_ENCODER = 0;
    public static int[] SHARED_MOTORS_RIGHT_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static int[] SHARED_MOTORS_LEFT_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static TecbotSpeedController.TypeOfMotor[] SHARED_RIGHT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};
    public static TecbotSpeedController.TypeOfMotor[] SHARED_LEFT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};
    /*
     * SHARED MOTORS ENDS
     */

    /*
     * CLIMBER STARTS
     */
    //LEFT WINCH
    public static int[] LEFT_WINCH_PORTS = {12};
    public static int[] LEFT_INVERTED_WINCH_PORTS = {};
    public static TecbotSpeedController.TypeOfMotor[] LEFT_WINCH_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX};

    //RIGHT WINCH
    public static final int[] RIGHT_WINCH_PORTS = {13};
    public static final int[] RIGHT_INVERTED_WINCH_PORTS = {};
    public static TecbotSpeedController.TypeOfMotor[] RIGHT_WINCH_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX};

    public static int[] GEAR_DISENGAGER_SOLENOID_PORTS = {0, 2, 3};
    public static final DoubleSolenoid.Value ENGAGED_SHOOTER_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DISENGAGED_SHOOTER_GEAR = DoubleSolenoid.Value.kReverse;

    public static final int CLIMBER_LIMIT_SWITCH_PORT = 0;
    /*
     * CLIMBER ENDS
     */

    /*
        SHOOTER STARTS
     */
    public static final int ANGLER_PORT = 5;
    public static double MANUAL_SHOOT = 0.4;
    public static boolean SHOOTER_ENCODER_IN_RIGHT_MOTOR = true;
    /*
        SHOOTER ENDS
     */

    /*
     * If any of the motors of the chassis must be inverted, indicate the port(s) in
     * these arrays.
     */

    /*
     * Intake Subsystem
     *
     */

    public static int[] FRONT_INTAKE_MOTORS = {1};
    public static int[] FRONT_INTAKE_MOTOR_DIRECTION = {};
    public static TypeOfMotor[] FRONT_INTAKE_MOTORS_TYPES = {TypeOfMotor.TALON_SRX};

    public static int[] REAR_INTAKE_MOTORS = {0};
    public static int[] REAR_INTAKE_MOTOR_DIRECTION = {};
    public static TypeOfMotor[] REAR_INTAKE_MOTORS_TYPES = {TypeOfMotor.TALON_SRX};


    public static int[] FRONT_SOLENOIDS = {0, 4, 5};
    public static int[] REAR_SOLENOIDS = {0, 6, 7};

    public static int servoPort = 0;

    /*
    Intake Subsystem ENDS
     */
    /*
        TransportationSystem Subsystems
     */
    public static final TypeOfMotor[] TRANSPORTATION_SYSTEM_TYPE_OF_MOTORS = {TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX};
    public static final int[] TRANSPORTATION_SYSTEM_MOTOR_PORTS = {3, 4, 5, 6};
    public static final int[] TRANSPORTATION_SYSTEM_INVERTED_MOTOR_PORTS = {};
    public static final int[] DEFLECTOR_SOLENOID = {1,6,7};
    /*
    TransportationSystem Subsystem ENDS
     */
}
