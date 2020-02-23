/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotMotorList;

public class Climber extends SubsystemBase {
    /**
     * Creates a new Climber.
     */
    TecbotMotorList leftWinchMotors;
    TecbotMotorList rightWinchMotors;
    DoubleSolenoid gearDisengager;
    int xWhenPressedCount;
    private boolean yCopilotPressedShrek = false;

    public boolean shrekPowerHasBeenActivated = false;

    public Climber() {

        leftWinchMotors = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_LEFT_WINCH_PORTS, RobotMap.CLIMBER_LEFT_INVERTED_WINCH_PORTS, RobotMap.CLIMBER_LEFT_WINCH_MOTOR_TYPES);
        rightWinchMotors = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_RIGHT_WINCH_PORTS, RobotMap.CLIMBER_RIGHT_INVERTED_WINCH_PORTS, RobotMap.CLIMBER_RIGHT_WINCH_MOTOR_TYPES);

        gearDisengager = RobotConfigurator.buildDoubleSolenoid(RobotMap.CLIMBER_GEAR_DISENGAGER_SOLENOID_PORTS);
        xWhenPressedCount = 0;

    }

    public void gearDisengagerToggle() {
        if (gearDisengager.get() == Value.kForward)
            gearDisengager.set(Value.kReverse);
        else
            gearDisengager.set(Value.kForward);
    }

    public void engageGear() {
        gearDisengager.set(RobotMap.CLIMBER_ENGAGED_SHOOTER_GEAR);
    }

    public void disengageGear() {
        gearDisengager.set(RobotMap.CLIMBER_DISENGAGED_SHOOTER_GEAR);
    }

    /**
     * @param leftSpeed  Requires double for the speed for lifting the left hook
     * @param rightSpeed Requires double for the speed for lifting the right hook
     */
    public void setWinchSpeed(double leftSpeed, double rightSpeed) {
        leftWinchMotors.setAll(leftSpeed);
        rightWinchMotors.setAll(rightSpeed);
    }

    /**
     * @param leftWinchSpeed speed to set to the left winch motors.
     */
    public void setLeftWinchSpeed(double leftWinchSpeed) {
        leftWinchMotors.setAll(leftWinchSpeed);
    }

    /**
     * @param rightWinchSpeed speed to set to the left winch motors.
     */
    public void setRightWinchSpeed(double rightWinchSpeed) {
        leftWinchMotors.setAll(rightWinchSpeed);
    }

    /**
     * @param pulleyPowerRight Requires double for right pulley speed
     * @param pulleyPowerLeft  Requires double for left pulley speed
     */
    public void setPulleySpeed(double pulleyPowerLeft, double pulleyPowerRight) {
        //reel, cannot do this if input is negative
        Robot.getRobotContainer().getSharedMotors().setAll(pulleyPowerLeft, pulleyPowerRight);
    }

    public void addToXCounter() {
        setxWhenPressedCount(getxWhenPressedCount() + 1);
        if (getxWhenPressedCount() == 2) shrekPowerHasBeenActivated = true;
    }

    public int getxWhenPressedCount() {
        return xWhenPressedCount;
    }

    public void setxWhenPressedCount(int xWhenPressedCount) {
        this.xWhenPressedCount = xWhenPressedCount;
    }

    public TecbotMotorList getLeftWinchMotors() {
        return leftWinchMotors;
    }

    public TecbotMotorList getRightWinchMotors() {
        return rightWinchMotors;
    }

    public boolean isyCopilotPressedShrek() {
        return yCopilotPressedShrek;
    }

    public void setyCopilotPressedShrek(boolean yCopilotPressedShrek) {
        this.yCopilotPressedShrek = yCopilotPressedShrek;
    }

    public void setTrueyCopilotPressed() {
        setyCopilotPressedShrek(true);
    }

    public void resetXCounter() {
        setxWhenPressedCount(0);
    }

    public void resetXCounterAfterTime(int seconds) {
        Notifier starter = new Notifier(this::resetXCounter);
        starter.startSingle(seconds);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
