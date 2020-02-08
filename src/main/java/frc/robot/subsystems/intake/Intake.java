/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.*;

public class Intake extends SubsystemBase {

    TecbotMotorList frontMotors;
    TecbotMotorList rearMotors;

    ArrayList<DoubleSolenoid> frontSolenoids;
    ArrayList<DoubleSolenoid> rearSolenoids;

    static HashMap<Color, Integer> controlPanelColors;
    static HashMap<Integer, Color> controlPanelIDs;

    Servo sensorServo;

    /**
     * Creates a new Intake.
     */
    public Intake() {

        frontMotors = RobotConfigurator.buildMotorList(RobotMap.FRONT_INTAKE_MOTORS, RobotMap.FRONT_INTAKE_MOTOR_DIRECTION, RobotMap.FRONT_INTAKE_MOTORS_TYPES);

        rearMotors = RobotConfigurator.buildMotorList(RobotMap.REAR_INTAKE_MOTORS, RobotMap.REAR_INTAKE_MOTOR_DIRECTION, RobotMap.REAR_INTAKE_MOTORS_TYPES);

        try {
            frontSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.FRONT_SOLENOIDS));
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            rearSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.REAR_SOLENOIDS));
        } catch (Exception e) {
            e.printStackTrace();
        }
        controlPanelColors = new HashMap<>();
        controlPanelColors.put(Color.RED, TecbotConstants.RED_ID);
        controlPanelColors.put(Color.BLUE, TecbotConstants.BLUE_ID);
        controlPanelColors.put(Color.GREEN, TecbotConstants.GREEN_ID);
        controlPanelColors.put(Color.YELLOW, TecbotConstants.YELLOW_ID);

        controlPanelIDs = new HashMap<>();
        controlPanelIDs.put(TecbotConstants.RED_ID, Color.RED);
        controlPanelIDs.put(TecbotConstants.BLUE_ID, Color.BLUE);
        controlPanelIDs.put(TecbotConstants.GREEN_ID, Color.GREEN);
        controlPanelIDs.put(TecbotConstants.YELLOW_ID, Color.YELLOW);

        sensorServo = RobotConfigurator.buildServo(RobotMap.servoPort);
    }

    /* Front Intake */

    public void frontIntakeForward() {
        frontMotors.setAll(TecbotConstants.FRONT_INTAKE_SPEED);
    }

    public void frontIntakeOff() {
        frontMotors.setAll(0);
    }

    public void frontIntakeReverse() {
        frontMotors.setAll(-TecbotConstants.FRONT_INTAKE_SPEED);
    }

    /* Rear Intake */

    public void rearIntakeForward() {
        rearMotors.setAll(TecbotConstants.REAR_INTAKE_SPEED);
    }

    public void rearIntakeOff() {
        rearMotors.setAll(0);
    }

    public void rearIntakeReverse() {
        rearMotors.setAll(-TecbotConstants.REAR_INTAKE_SPEED);
    }

    public void setRawFrontIntake(double speed) {
        frontMotors.setAll(speed);
    }

    public void setRawRearIntake(double speed) {
        rearMotors.setAll(speed);
    }

    /* Front Solenoids */

    public void frontIntakeSolenoidOn() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(Value.kForward);
        }
    }

    public void frontIntakeSolenoidOff() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(Value.kReverse);
        }
    }
    /* Rear Solenoids */

    public void rearIntakeSolenoidOn() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(Value.kForward);
        }
    }

    public void rearIntakeSolenoidOff() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(Value.kReverse);
        }
    }

    public void setServoPosition(int angle){
        sensorServo.set(angle);
    }
    public double getServoPosition(){
        return sensorServo.getAngle();
    }

    public enum Color {RED, GREEN, BLUE, YELLOW}

    public static int getIDFromColor(Color color){

        return controlPanelColors.get(color);
    }

    public static Color getColorFromID(int id){
        return controlPanelIDs.get(id);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
