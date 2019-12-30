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

    // The equivalence between meters to encoder count
	// Meter * meters_to_encoder = encoder count
	public static float K_METERS_TO_ENCODER = (float) (30000 / (.2034 * Math.PI));
	// public static float k_meters_to_encoder = (float)
	// (RobotMap.k_tic_per_revolution / (RobotMap.k_wheel_diameter * Math.PI));
	public static int K_CHASSIS_TIC_PER_REVOLUTION = 30000;
	public static float K_WHEEL_DIAMETER = .2032f;
}