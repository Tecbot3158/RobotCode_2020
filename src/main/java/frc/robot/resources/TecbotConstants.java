package frc.robot.resources;

public class TecbotConstants {

    // ﻿﻿﻿﻿﻿﻿ -3509.97021484375 ﻿
    //  ﻿﻿﻿﻿﻿﻿ -3524.081787109375 ﻿

    public static final double CHASSIS_TURN_MAX_DISTANCE = 90;
    public static final double CHASSIS_TURN_ARRIVE_OFFSET = 10;

    public static final double CHASSIS_STRAIGHT_MAX_DISTANCE = 20;
    public static final double CHASSIS_STRAIGHT_ARRIVE_OFFSET = 2;

    // Constants used to calculate speed reduction control in swerve mode
    // Units in meters
    public static final double CHASSIS_SWERVE_MAX_DISTANCE = 20;
    public static final double CHASSIS_SWERVE_ARRIVE_OFFSET = 2;


    public static final double TURN_CORRECTION = .01;
    // The distance in angles that it needs to be from target in order to be
    // considered onTarget
    public static final double QUICK_TURN_OFFSET = 20;

    public static final double QUICK_TURN_CORRECTION = .05;
    // The sides move at a different speed than the middle wheel, so this constant
    // controls that difference to try to
    // Make them move at the same speed
    public static final double MIDDLE_SIDES_CORRECTION = .6;

    public static final double SPLINE_REDUCING_SPEED_CONSTANT = .8;
    public static final double SPLINE_TURN_CORRECTION = .05;

    public static final double K_DISTANCE_BETWEEN_WHEELS = .53975;

    public static final double K_CHASSIS_WHEEL_DIAMETER = .3048;
    public static final double K_CHASSIS_TIC_PER_REVOLUTION = 22;

    public static final double K_MIDDLE_WHEEL_DIAMETER = .2032;
    public static final double K_MIDDLE_WHEEL_TIC_PER_REVOLUTION = 8.75;

    // The equivalence between meters to encoder count

    // (encoderCount / tickPerRevolution) * wheelDiam * Pi = distance


    // K_CHASSIS_ENCODER_TO_METERS * encoder count = distance in meters
    // (encoderCount / tickPerRevolution) * wheelDiam * Pi = distance
    // encoderCount (1/tickPerRevolution)*wheelDiam * Pi = distance
    // Encoder to meters = 1/tickPerRevolution * wheelDiam * Pi
    // This one works only for the side wheels
    public static final double K_CHASSIS_ENCODER_TO_METERS = (1 / K_CHASSIS_TIC_PER_REVOLUTION) * K_CHASSIS_WHEEL_DIAMETER
            * Math.PI;
    // This one works only for the middle wheel
    public static final double K_MIDDLE_WHEEL_ENCODER_TO_METERS = (1 / K_MIDDLE_WHEEL_TIC_PER_REVOLUTION)
            * K_MIDDLE_WHEEL_TIC_PER_REVOLUTION * Math.PI;

    // K_METERS_TO_ENCODER * meters = encoderCount
    // (encoderCount / tickPerRevolution) * wheelDiam * Pi = distance
    // (encoderCount * wheel diam*pi) / tickPerRevolution  = distance
    // encoderCount * wheel diam * pi = distance * tickPerRevolution
    // encoderCount = (distance * tickPerRevolution) / (wheel diam * pi)

    // This only works for chassis wheels
    public static final double K_CHASSIS_METERS_TO_ENCODER = (float) (K_CHASSIS_TIC_PER_REVOLUTION / (K_CHASSIS_WHEEL_DIAMETER * Math.PI));
    // This only works for the middle wheel
    public static final double K_MIDDLE_WHEEL_METERS_TO_ENCODER = (float) (K_MIDDLE_WHEEL_TIC_PER_REVOLUTION / (K_MIDDLE_WHEEL_DIAMETER * Math.PI));


    public static final double K_STRAIGHT_P = 0;
    public static final double K_STRAIGHT_I = 0;
    public static final double K_STRAIGHT_D = 0;
    public static final double K_PID_STRAIGHT_ARRIVE_OFFSET = 0;

    public static final double K_TURN_P = 0;
    public static final double K_TURN_I = 0;
    public static final double K_TURN_D = 0;
    public static final double K_PID_TURN_ARRIVE_OFFSET = 0;

    // The equivalence between meters to encoder count
    // Meter * meters_to_encoder = encoder count
    public static final double K_METERS_TO_ENCODER = (float) (30000 / (.2034 * Math.PI));

    //SHOOTER
    /*

    public static final double SHOOTER_TRENCH_SHOOTING_SPEED = 0.75;
    public static final double SHOOTER_INITIATION_LINE_SHOOTING_SPEED = 0.97;
    public static final double SHOOTER_TARGET_ZONE_SHOOTING_SPEED = 0.75;
     */
    //used to be 0.75
    public static final double SHOOTER_TRENCH_SHOOTING_SPEED = 0.75;
    //used to be 0.97
    public static final double SHOOTER_INITIATION_LINE_SHOOTING_SPEED = 0.97;
    //used to be 0.75
    public static final double SHOOTER_TARGET_ZONE_SHOOTING_SPEED = 0.75;
    public static final double SHOOTER_OFF = 0;

    public static final double SHOOTER_AUTONOMOUS_SPEED_DR01D3K4 = 0.85;
    public static final double SHOOTER_AUTONOMOUS_SPEED_SHOOT_3PCs_N_MOVE = 0.79;

    //POWER SH
    public static final double TRENCH_SHOOTING_POWER = 0;
    public static final double INITIATION_LINE_SHOOTING_POWER = 0;
    public static final double TARGET_ZONE_SHOOTING_POWER = 0;

    //0 TO 1
    public static final double TRENCH_SHOOTING_ANGLE = 0;
    public static final double INITIATION_LINE_SHOOTING_ANGLE = 0;
    public static final double TARGET_ZONE_SHOOTING_ANGLE = 0;
    public static final double SHOOTER_OFF_ANGLE = 0;

    public static final double K_SHOOTER_P = 1;
    public static final double K_SHOOTER_I = 0;
    public static final double K_SHOOTER_D = 0;


    public static final double K_SHOOTER_FREE_RPS = 3390 / 60.0; // determined by santiaGo

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double KS_VOLTS = 0.1;
    public static final double KV_VOLT_SECONDS_PER_ROTATION =
            // Should have value 12V at free speed...
            12.0 / K_SHOOTER_FREE_RPS;


    //SHOOTER ENDS

    //CLIMBER STARTS
    public static final double WINCH_SPEED = 1;
    public static final double WINCH_LOOSEN_ROPE_DEFAULT_SPEED = 0.5;
    public static final double WINCH_LOOSEN_ROPE_DEFAULT_TIME = 0.20;
    //CLIMBER ENDS

    //INTAKE STARTS
    public static final double FRONT_INTAKE_SPEED = 1;
    public static final double REAR_INTAKE_SPEED = 1;
    //INTAKE ENDS

    //CONTROL PANEL STARTS

    //The ids assigned to the colors will be used to know in which order they are

    public static final int CONTROL_PANEL_RED_ID = 1;
    public static final int CONTROL_PANEL_GREEN_ID = 2;
    public static final int CONTROL_PANEL_BLUE_ID = 3;
    public static final int CONTROL_PANEL_YELLOW_ID = 4;

    public static final int CONTROL_PANEL_SENSOR_RETRACTED_ANGLE = 35;
    public static final int CONTROL_PANEL_SENSOR_EXTENDED_ANGLE = 170;


    //CONTROL PANEL ENDS

    //TRANSPORTATION SYSTEMS STARTS
    public static final double TRANSPORTATION_SYSTEM_POWER = 0.4;
    public static final double TRANSPORTATION_SYSTEM_SHOOTING_POWER = 0.7;
    public static final double TRANSPORTATION_SYSTEM_REVERSE_TIME_COMPENSATION_IN_SECONDS = 0.5;
    //TRANSPORTATION SYSTEMS ENDS

    // JOYSTICK STARTS
    public static final double DEFAULT_JOYSTICK_OFFSET = .1;
    public static final double JOYSTICK_SPEED_RELEASE_POINT = .5;
    public static final double JOYSTICK_SPEED_MULTIPLIER = .45;
    public static final boolean APPLY_SPEED_RELEASE_TO_LEFT_AXIS = true;

}