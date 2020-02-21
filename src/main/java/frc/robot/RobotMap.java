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


    /*
     * CHASSIS / DRIVE TRAIN STARTS
     */

    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_PORTS = {1, 2};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_PORTS = {9, 10};

    public static final TypeOfMotor[] DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};
    public static final TypeOfMotor[] DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};


    /*
     * If encoder is connected to speed controller,
     * indicate the speed controller port here, and put encoder ports in
     * config not set
     */
    public static final int DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_WITH_ENCODER = 1;
    public static final int DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_WITH_ENCODER = 9;
    public static final int DRIVE_TRAIN_MIDDLE_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;

    /*
     * If encoder is connected to RoboRIO,
     * indicate the port here, and put encoder motor ports in
     * config not set
     */
    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};

    /*
     * If any of the motors of the chassis must be inverted,
     * indicate the port(s) in these arrays.
     */
    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_INVERTED_MOTORS = {};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_INVERTED_MOTORS = {};

    public static final boolean DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_IS_INVERTED = true;
    public static final boolean DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_IS_INVERTED = false;
    public static final boolean DRIVE_TRAIN_MIDDLE_CHASSIS_ENCODER_IS_INVERTED = false;

    public static final int[] DRIVE_TRAIN_TRANSMISSION_SOLENOID_PORTS = {1, 4, 5};
    public static final DoubleSolenoid.Value DRIVE_TRAIN_TORQUE_TRANSMISSION = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DRIVE_TRAIN_SPEED_TRANSMISSION = DoubleSolenoid.Value.kReverse;
    public static final boolean DRIVE_TRAIN_TRANSMISSION_AVAILABLE = true;

    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_PORT = {11};
    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_INVERTED_MOTORS = {};
    public static final TypeOfMotor[] DRIVE_TRAIN_MIDDLE_WHEEL_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS};

    public static final int[] DRIVE_TRAIN_WHEEL_SOLENOID_PORTS = {1, 6, 7};
    public static final DoubleSolenoid.Value DRIVE_TRAIN_LOWERED_WHEEL = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DRIVE_TRAIN_RAISED_WHEEL = DoubleSolenoid.Value.kReverse;
    public static final boolean DRIVE_TRAIN_DRAGON_FLY_IS_AVAILABLE = false;

    /*
        CHASSIS / DRIVE TRAIN
     */

    /*
     * SHARED MOTORS STARTS
     */
    public static final int[] SHARED_MOTORS_LEFT_PORTS = {5, 6};
    public static final int[] SHARED_MOTORS_RIGHT_PORTS = {14, 12};
    public static final int[] SHARED_MOTORS_LEFT_INVERTED_MOTORS = {};
    public static final int[] SHARED_MOTORS_RIGHT_INVERTED_MOTORS = {14, 12};
    // This integer will contain the motor port which the shooter encoder is connected to.
    public static final int SHARED_MOTORS_RIGHT_ENCODER_MOTOR_PORT = SHARED_MOTORS_RIGHT_PORTS[1];
    // This integer will contain the motor port which the shooter encoder is connected to.
    public static final int SHARED_MOTORS_LEFT_MOTOR_WITH_ENCODER = 0;
    /**
     * This is the port for the right shared motors encoder itself, in this case it is CONFIG_NOT_SET {@link RobotConfigurator},
     * since it is directly connected to a TalonSRX
     */
    public static final int[] SHARED_MOTORS_RIGHT_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    /**
     * This is the port for the left shared motors encoder itself, in this case it is CONFIG_NOT_SET {@link RobotConfigurator},
     * since it is directly connected to a TalonSRX
     */
    //TODO set SHOOTER_ENCODER_IN_RIGHT_MOTOR boolean
    public static final int[] SHARED_MOTORS_LEFT_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static final TecbotSpeedController.TypeOfMotor[] SHARED_MOTORS_RIGHT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};
    public static final TecbotSpeedController.TypeOfMotor[] SHARED_MOTORS_LEFT_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX, TecbotSpeedController.TypeOfMotor.TALON_SRX};
    /*
     * SHARED MOTORS ENDS
     */

    /*
     * CLIMBER STARTS
     */
    //LEFT WINCH
    public static final int[] CLIMBER_LEFT_WINCH_PORTS = {8};
    public static final int[] CLIMBER_LEFT_INVERTED_WINCH_PORTS = {};
    public static final TecbotSpeedController.TypeOfMotor[] CLIMBER_LEFT_WINCH_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX};

    //RIGHT WINCH
    public static final int[] CLIMBER_RIGHT_WINCH_PORTS = {13};
    public static final int[] CLIMBER_RIGHT_INVERTED_WINCH_PORTS = {};
    public static final TecbotSpeedController.TypeOfMotor[] CLIMBER_RIGHT_WINCH_MOTOR_TYPES = {TecbotSpeedController.TypeOfMotor.TALON_SRX};

    public static final int[] CLIMBER_GEAR_DISENGAGER_SOLENOID_PORTS = {1, 0, 1};
    public static final DoubleSolenoid.Value CLIMBER_ENGAGED_SHOOTER_GEAR = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value CLIMBER_DISENGAGED_SHOOTER_GEAR = DoubleSolenoid.Value.kReverse;

    public static final int CLIMBER_LIMIT_SWITCH_PORT = 2;
    /*
     * CLIMBER ENDS
     */

    /*
        SHOOTER STARTS
     */
    public static final int SHOOTER_ANGLER_PORT = 5;
    public static final double SHOOTER_MANUAL_SHOOT = 0.4;
    public static final boolean SHOOTER_ENCODER_IN_RIGHT_MOTOR = true;
    /*
        SHOOTER ENDS
     */

    /*
     * Intake Subsystem
     *
     */
    public static final int[] FRONT_INTAKE_MOTOR_PORTS = {1};
    public static final int[] FRONT_INTAKE_INVERTED_MOTOR_PORTS = {};
    public static final TypeOfMotor[] FRONT_INTAKE_MOTOR_TYPES = {TypeOfMotor.TALON_SRX};

    public static final int[] REAR_INTAKE_MOTOR_PORTS = {0};
    public static final int[] REAR_INTAKE_INVERTED_MOTOR_PORTS = {};
    public static final TypeOfMotor[] REAR_INTAKE_MOTOR_TYPES = {TypeOfMotor.TALON_SRX};


    public static final int[] FRONT_INTAKE_SOLENOID_PORTS = {1, 2, 3};
    public static final int[] REAR_INTAKE_SOLENOID_PORTS = {2, 4, 5};

    public static final int COLOR_SENSOR_SERVO_PORT = 0;

    /*
    Intake Subsystem ENDS
     */
    /*
        TransportationSystem Subsystems
     */
    public static final TypeOfMotor[] TRANSPORTATION_SYSTEM_TYPE_OF_MOTORS = {TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX, TypeOfMotor.TALON_SRX};
    public static final int[] TRANSPORTATION_SYSTEM_MOTOR_PORTS = {3, 4, 5, 6};
    public static final int[] TRANSPORTATION_SYSTEM_INVERTED_MOTOR_PORTS = {};
    public static final int[] DEFLECTOR_SOLENOID_PORTS = {1, 6, 7};
    public static final int TRANSPORTATION_SYSTEM_INFRARED_INTAKE_SENSOR_PORT = 0;
    public static final int TRANSPORTATION_SYSTEM_INFRARED_SHOOTER_SENSOR_PORT = 1;
    /*
    TransportationSystem Subsystem ENDS
     */
}
