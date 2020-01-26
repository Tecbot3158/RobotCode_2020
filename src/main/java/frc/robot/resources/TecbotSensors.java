package frc.robot.resources;

import frc.robot.RobotMap;
import frc.robot.subsystems.chassis.DriveTrain;

public class TecbotSensors {

    public static Navx tecbotGyro;

    public static TecbotEncoder leftChassisEncoder, rightChassisEncoder, middleChassisEncoder;

    public static void initializeAllSensors(){

        tecbotGyro = new Navx();
        leftChassisEncoder = RobotConfigurator.buildEncoder(DriveTrain.getLeftEncoderMotor(), RobotMap.leftChassisEncoderPorts[0],
                RobotMap.leftChassisEncoderPorts[1]);
        rightChassisEncoder = RobotConfigurator.buildEncoder(DriveTrain.getRightEncoderMotor(), RobotMap.rightChassisEncoderPorts[0],
                RobotMap.rightChassisEncoderPorts[1]);
        middleChassisEncoder = RobotConfigurator.buildEncoder(DriveTrain.getMiddleWheelMotor(), RobotMap.middleWheelEncoderPorts[0],
                RobotMap.middleWheelEncoderPorts[1]);

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

    public static double getEncoderRaw(SubsystemType subsystem){
        switch (subsystem){
            case RIGHT_CHASSIS:
                return rightChassisEncoder.getRaw();
            case LEFT_CHASSIS:
                return leftChassisEncoder.getRaw();
            case MIDDLE_CHASSIS:
                return middleChassisEncoder.getRaw();
            default:
                return -1;
        }
    }
    public enum SubsystemType {
        MIDDLE_CHASSIS, RIGHT_CHASSIS, LEFT_CHASSIS
    }

}
