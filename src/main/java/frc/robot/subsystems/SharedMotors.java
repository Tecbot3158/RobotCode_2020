package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotMotorList;
import frc.robot.resources.TecbotSpeedController;

public class SharedMotors {
    public TecbotMotorList rightSharedMotors, leftSharedMotors;
    private int encoderMotorRight = RobotMap.SHARED_RIGHT_MOTOR_WITH_ENCODER;
    private int encoderMotorLeft = RobotMap.SHARED_LEFT_MOTOR_WITH_ENCODER;

    public SharedMotors() {
        rightSharedMotors = RobotConfigurator.buildMotorList(RobotMap.SHARED_MOTORS_RIGHT_PORTS, RobotMap.INVERTED_SHARED_MOTORS_RIGHT, RobotMap.SHARED_RIGHT_MOTOR_TYPES);
        leftSharedMotors = RobotConfigurator.buildMotorList(RobotMap.SHARED_MOTORS_LEFT_PORTS, RobotMap.INVERTED_SHARED_MOTORS_LEFT, RobotMap.SHARED_LEFT_MOTOR_TYPES);
    }

    /**
     * @param speedRight speed for right motors
     * @param speedLeft  speed for left motors
     */
    public void setAll(double speedLeft, double speedRight) {
        leftSharedMotors.setAll(speedLeft);
        rightSharedMotors.setAll(speedRight);
    }

    public TecbotSpeedController getMotorWithEncoderRight() {
        return (encoderMotorRight > 0) ? rightSharedMotors.getSpecificMotor(RobotMap.SHARED_RIGHT_MOTOR_WITH_ENCODER) : null;
    }

    public TecbotSpeedController getMotorWithEncoderLeft() {
        return (encoderMotorLeft > 0) ? leftSharedMotors.getSpecificMotor(RobotMap.SHARED_LEFT_MOTOR_WITH_ENCODER) : null;
    }
}
