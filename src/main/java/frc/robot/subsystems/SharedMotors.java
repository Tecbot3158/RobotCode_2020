package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

import java.util.ArrayList;
import java.util.List;

public class SharedMotors {
    public List<TecbotSpeedController> rightSharedMotors, leftSharedMotors;
    private int encoderMotorRight = RobotMap.SHARED_RIGHT_MOTOR_WITH_ENCODER;
    private int encoderMotorLeft = RobotMap.SHARED_LEFT_MOTOR_WITH_ENCODER;

    public void initializeSharedMotors() {
        rightSharedMotors = new ArrayList<>();
        leftSharedMotors = new ArrayList<>();
        for (int i = 0; i < RobotMap.SHARED_MOTORS_RIGHT_PORTS.length; i++) {
            rightSharedMotors.add(new TecbotSpeedController(RobotMap.SHARED_MOTORS_RIGHT_PORTS[i],
                    RobotMap.SHARED_RIGHT_MOTOR_TYPES[i]));
            for (int port : RobotMap.INVERTED_SHARED_MOTORS_RIGHT) {
                if (port == RobotMap.SHARED_MOTORS_RIGHT_PORTS[i])
                    rightSharedMotors.get(i).setInverted(true);
            }
        }
        for (int i = 0; i < RobotMap.SHARED_MOTORS_LEFT_PORTS.length; i++) {
            leftSharedMotors.add(new TecbotSpeedController(RobotMap.SHARED_MOTORS_LEFT_PORTS[i],
                    RobotMap.SHARED_LEFT_MOTOR_TYPES[i]));
            for (int port : RobotMap.INVERTED_SHARED_MOTORS_LEFT) {
                if (port == RobotMap.SHARED_MOTORS_LEFT_PORTS[i])
                    leftSharedMotors.get(i).setInverted(true);
            }
        }
    }

    /**
     * @param speedRight speed for right motors
     * @param speedLeft  speed for left motors
     */
    public void setAll(double speedRight, double speedLeft) {
        for (TecbotSpeedController motor : rightSharedMotors) {
            motor.set(speedRight);
        }
        for (TecbotSpeedController motor : leftSharedMotors) {
            motor.set(speedLeft);
        }
    }

    public TecbotSpeedController getMotorWithEncoderRight() {
        return (encoderMotorRight > 0) ? rightSharedMotors.get(encoderMotorRight) : null;
    }

    public TecbotSpeedController getMotorWithEncoderLeft() {
        return (encoderMotorLeft > 0) ? leftSharedMotors.get(encoderMotorLeft) : null;
    }
}