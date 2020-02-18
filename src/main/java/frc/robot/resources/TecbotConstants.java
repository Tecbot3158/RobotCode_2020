package frc.robot.resources;

public class TecbotConstants {

    public static final double CHASSIS_TURN_MAX_DISTANCE = 20;
    public static final double CHASSIS_TURN_ARRIVE_OFFSET = 2;

    public static final double CHASSIS_STRAIGHT_MAX_DISTANCE = 20;
    public static final double CHASSIS_STRAIGHT_ARRIVE_OFFSET = 2;

    public static final double TURN_CORRECTION = .05;
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

    public static final double K_DISTANCE_BETWEEN_WHEELS = .54;
    public static final double K_CHASSIS_WHEEL_DIAMETER = .34;

    public static final int K_CHASSIS_TIC_PER_REVOLUTION = 30000;
    public static final double K_MIDDLE_WHEEL_DIAMETER = .34;
    public static final int K_MIDDLE_WHEEL_TIC_PER_REVOLUTION = 30000;

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

    //SHOOTER STARTS
    public static final double TRENCH_SHOOTING_SPEED = 0;
    public static final double INITIATION_LINE_SHOOTING_SPEED = 0;
    public static final double TARGET_ZONE_SHOOTING_SPEED = 0;
    public static final double SHOOTER_OFF = 0;

    //0 TO 1
    public static final double TRENCH_SHOOTING_ANGLE = 0;
    public static final double INITIATION_LINE_SHOOTING_ANGLE = 0;
    public static final double TARGET_ZONE_SHOOTING_ANGLE = 0;
    public static final double SHOOTER_OFF_ANGLE = 0;

    public static final double K_SHOOTER_P = 0;
    public static final double K_SHOOTER_I = 0;
    public static final double K_SHOOTER_D = 0;
    //SHOOTER ENDS

    //CLIMBER STARTS
    public static final double WINCH_SPEED = 1;
    //CLIMBER ENDS

    //INTAKE STARTS
    public static final double FRONT_INTAKE_SPEED = 0.8;
    public static final double REAR_INTAKE_SPEED = 0.8;
    //INTAKE ENDS

    //CONTROL PANEL STARTS

    //The ids assigned to the colors will be used to know in which order they are

    public static final int CONTROL_PANEL_RED_ID = 1;
    public static final int CONTROL_PANEL_GREEN_ID = 2;
    public static final int CONTROL_PANEL_BLUE_ID = 3;
    public static final int CONTROL_PANEL_YELLOW_ID = 4;

    public static final int CONTROL_PANEL_SENSOR_RETRACTED_ANGLE = 0;
    public static final int CONTROL_PANEL_SENSOR_EXTENDED_ANGLE = 0;


    //CONTROL PANEL ENDS

    //TRANSPORTATION SYSTEMS STARTS
    public static final double TRANSPORTATION_SYSTEM_POWER = 0.2;
    //TRANSPORTATION SYSTEMS ENDS

}