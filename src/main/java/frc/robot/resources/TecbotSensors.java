package frc.robot.resources;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class TecbotSensors {

    public static Navx tecbotGyro;
    public static TecbotEncoder sharedMotorsRightEncoder, sharedMotorsLeftEncoder;

    public static void initializeAllSensors() {

        tecbotGyro = new Navx();
        sharedMotorsRightEncoder = RobotConfigurator.buildEncoder(RobotContainer.sharedMotors.getMotorWithEncoderRight(),
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[0],
        RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[1]);
        sharedMotorsLeftEncoder = RobotConfigurator.buildEncoder(RobotContainer.sharedMotors.getMotorWithEncoderLeft(),
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[1]);

    }

    public static void sensorsPeriodic() {
        tecbotGyro.run();
    }

    public static Navx getTecbotGyro() {
        return tecbotGyro;
    }

    public static double getYaw() {
        return tecbotGyro.getYaw();
    }

    public static double getEncoderRaw(SubsystemType subsystem){
        switch (subsystem){
            case SHARED_RIGHT:
                return sharedMotorsRightEncoder.getRaw();
            case SHARED_LEFT:
                return sharedMotorsLeftEncoder.getRaw();
            default:
                return -1;
        }
    }

    public enum SubsystemType {
        SHARED_RIGHT, SHARED_LEFT
    }

}
