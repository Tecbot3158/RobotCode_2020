package frc.robot.resources;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.chassis.DriveTrain;

public class TecbotSensors {

    public static Navx tecbotGyro;

    public static TecbotEncoder leftChassisEncoder, rightChassisEncoder, middleChassisEncoder;
    public static TecbotEncoder shooterEncoder;

    //Color sensors

    public static ColorSensorV3 colorSensorV3;
    public static ColorMatch colorMatcher;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;
    public final static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public static void initializeAllSensors(){

        tecbotGyro = new Navx();

        shooterEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderLeft() , RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET);
        

        TecbotSensors.colorSensorV3 = new ColorSensorV3(i2cPort);
        TecbotSensors.colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);

    }

    public static void sensorsPeriodic(){
        tecbotGyro.run();
    }

    public static Navx getTecbotGyro(){
        return tecbotGyro;
    }

    public static double getYaw(){
        return tecbotGyro.getYaw();
    }

    public static String getColor() {
        final ColorMatchResult match = colorMatcher.matchClosestColor(colorSensorV3.getColor());

        if (match.color == kBlueTarget) {
            return "Blue";
        } else if (match.color == kRedTarget) {
            return "Red";
        } else if (match.color == kGreenTarget) {
            return "Green";
        } else if (match.color == kYellowTarget) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    /**
     * @return Raw value form selectd encoder
     */
    public static double getEncoderRaw(SubsystemType subsystem){
        switch (subsystem){
            case RIGHT_CHASSIS:
                return rightChassisEncoder.getRaw();
            case LEFT_CHASSIS:
                return leftChassisEncoder.getRaw();
            case MIDDLE_CHASSIS:
                return middleChassisEncoder.getRaw();
            case SHOOTER: 
                return shooterEncoder.getRaw();
            default:
                return -1;
        }
    }
    public enum SubsystemType {
        MIDDLE_CHASSIS, RIGHT_CHASSIS, LEFT_CHASSIS, SHOOTER
    }

}