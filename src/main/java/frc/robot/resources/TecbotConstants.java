package frc.robot.resources;

public class TecbotConstants {

	public static final double CHASSIS_TURN_MAX_DISTANCE = 20;
	public static final double CHASSIS_TURN_ARRIVE_OFFSET = 2;

	public static final double TURN_CORRECTION = .05;
	// The distance in angles that it needs to be from target in order to be considered onTarget
    public static final double QUICK_TURN_OFFSET = 20;
	public static final double QUICK_TURN_CORRECTION = .05;
	//The sides move at a different speed than the middle wheel, so this constant controls that difference to try to
	//Make them move at the same speed
	public static final double MIDDLE_SIDES_CORRECTION = .6;

	public static final double SPLINE_REDUCING_SPEED_CONSTANT = .8;
	public static final double SPLINE_TURN_CORRECTION = .05;


	public static final double K_DISTANCE_BETWEEN_WHEELS = .54;
	public static final double K_CHASSIS_WHEEL_DIAMETER = .34;
	public static int K_CHASSIS_TIC_PER_REVOLUTION = 30000;
	public static final double K_MIDDLE_WHEEL_DIAMETER = .34;
	public static int K_MIDDLE_WHEEL_TIC_PER_REVOLUTION = 30000;

	// The equivalence between meters to encoder count
	// Meter * meters_to_encoder = encoder count
	public static double K_METERS_TO_ENCODER = (float) (30000 / (.2034 * Math.PI));

	//K_CHASSIS_ENCODER_TO_METERS * encoder count = distance in meters
	// (encoderCount / tickPerRevolution) * wheelDiam * Pi = distance
	// encoderCount (1/tickPerRevolution)*wheelDiam * Pi = distance
	// Encoder to meters = 1/tickPerRevolution * wheelDiam * Pi
	// This one works only for the side wheels
	public static double K_CHASSIS_ENCODER_TO_METERS = (1/K_CHASSIS_TIC_PER_REVOLUTION) * K_CHASSIS_WHEEL_DIAMETER * Math.PI;
	/**
	 * This one works only for the middle wheel
	 * 
	 */
	public static double K_MIDDLE_WHEEL_ENCODER_TO_METERS = (1/K_MIDDLE_WHEEL_TIC_PER_REVOLUTION) * K_MIDDLE_WHEEL_TIC_PER_REVOLUTION * Math.PI;
    
	public static final double TRENCH_SHOOTING_SPEED = 0;
	public static final double INITIATION_LINE_SHOOTING_SPEED = 0;
	public static final double LOADING_BAY_SHOOTING_SPEED = 0;

	public static final double TRENCH_SHOOTING_ANGLE = 0;
	public static final double INITIATION_LINE_SHOOTING_ANGLE = 0;
	public static final double LOADING_BAY_SHOOTING_ANGLE = 0;


}