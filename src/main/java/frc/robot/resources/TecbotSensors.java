package frc.robot.resources;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TecbotSensors {

    private Navx tecbotGyro;

    private TecbotEncoder leftChassisEncoder, rightChassisEncoder, middleChassisEncoder,
            sharedMotorsRightEncoder, sharedMotorsLeftEncoder;

    //Color sensor
    private ColorSensorV3 colorSensorV3;
    private ColorMatch colorMatcher;
    //Color sensor constants
    private final I2C.Port I2C_PORT_ONBOARD = I2C.Port.kOnboard;
    private final Color K_BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color K_GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color K_RED_TARGET = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color K_YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public TecbotSensors() {

    }

    public void initializeAllSensors() {

        tecbotGyro = new Navx();
        sharedMotorsRightEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderRight(),
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[1]);
        sharedMotorsLeftEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderLeft(),
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[1]);

        leftChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.LEFT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.LEFT_CHASSIS_ENCODER_PORTS[0], RobotMap.LEFT_CHASSIS_ENCODER_PORTS[1]);
        rightChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.RIGHT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.RIGHT_CHASSIS_ENCODER_PORTS[0], RobotMap.RIGHT_CHASSIS_ENCODER_PORTS[1]);
        middleChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.MIDDLE_CHASSIS_MOTOR_WITH_ENCODER)
                        , RobotMap.MIDDLE_WHEEL_ENCODER_PORTS[0], RobotMap.MIDDLE_WHEEL_ENCODER_PORTS[1]);

        colorSensorV3 = new ColorSensorV3(I2C_PORT_ONBOARD);
        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(K_BLUE_TARGET);
        colorMatcher.addColorMatch(K_GREEN_TARGET);
        colorMatcher.addColorMatch(K_RED_TARGET);
        colorMatcher.addColorMatch(K_YELLOW_TARGET);

    }

    public void sensorsPeriodic() {
        tecbotGyro.run();
    }

    public Navx getTecbotGyro() {
        return tecbotGyro;
    }

    public double getYaw() {
        return tecbotGyro.getYaw();
    }

    public CurrentColor getColor() {
        final ColorMatchResult match = colorMatcher.matchClosestColor(colorSensorV3.getColor());

        if (match.color == K_BLUE_TARGET) {
            return CurrentColor.BLUE;
        } else if (match.color == K_RED_TARGET) {
            return CurrentColor.RED;
        } else if (match.color == K_GREEN_TARGET) {
            return CurrentColor.GREEN;
        } else if (match.color == K_YELLOW_TARGET) {
            return CurrentColor.YELLOW;
        } else {
            return null;
        }
    }

    public enum CurrentColor {
        RED,
        GREEN,
        BLUE,
        YELLOW
    }

    /**
     * @return Raw value form selected encoder
     */
    public double getEncoderRaw(SubsystemType subsystem) {
        switch (subsystem) {
            case RIGHT_CHASSIS:
                return rightChassisEncoder.getRaw();
            case LEFT_CHASSIS:
                return leftChassisEncoder.getRaw();
            case MIDDLE_CHASSIS:
                return middleChassisEncoder.getRaw();
            case SHOOTER:
                return RobotMap.SHOOTER_ENCODER_IN_RIGHT_MOTOR ?
                        rightChassisEncoder.getRaw() :
                        leftChassisEncoder.getRaw();
            default:
                return 0;
        }
    }

    public enum SubsystemType {
        MIDDLE_CHASSIS, RIGHT_CHASSIS, LEFT_CHASSIS, SHOOTER
    }

}
