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

        rearSolenoids = new ArrayList<>();
        frontSolenoids = new ArrayList<>();

        frontMotors = RobotConfigurator.buildMotorList(RobotMap.FRONT_INTAKE_MOTOR_PORTS, RobotMap.FRONT_INTAKE_INVERTED_MOTOR_PORTS, RobotMap.FRONT_INTAKE_MOTOR_TYPES);

        rearMotors = RobotConfigurator.buildMotorList(RobotMap.REAR_INTAKE_MOTOR_PORTS, RobotMap.REAR_INTAKE_INVERTED_MOTOR_PORTS, RobotMap.REAR_INTAKE_MOTOR_TYPES);

        frontSolenoids = new ArrayList<>();
        rearSolenoids = new ArrayList<>();

        frontSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.FRONT_INTAKE_SOLENOID_PORTS));

        rearSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.REAR_INTAKE_SOLENOID_PORTS));

        controlPanelColors = new HashMap<>();
        controlPanelColors.put(Color.RED, TecbotConstants.CONTROL_PANEL_RED_ID);
        controlPanelColors.put(Color.BLUE, TecbotConstants.CONTROL_PANEL_BLUE_ID);
        controlPanelColors.put(Color.GREEN, TecbotConstants.CONTROL_PANEL_GREEN_ID);
        controlPanelColors.put(Color.YELLOW, TecbotConstants.CONTROL_PANEL_YELLOW_ID);

        controlPanelIDs = new HashMap<>();
        controlPanelIDs.put(TecbotConstants.CONTROL_PANEL_RED_ID, Color.RED);
        controlPanelIDs.put(TecbotConstants.CONTROL_PANEL_BLUE_ID, Color.BLUE);
        controlPanelIDs.put(TecbotConstants.CONTROL_PANEL_GREEN_ID, Color.GREEN);
        controlPanelIDs.put(TecbotConstants.CONTROL_PANEL_YELLOW_ID, Color.YELLOW);

        sensorServo = RobotConfigurator.buildServo(RobotMap.COLOR_SENSOR_SERVO_PORT);
    }

    /* Front Intake */

    public void frontIntakeForward() {
        //frontMotors.setAll(TecbotConstants.FRONT_INTAKE_SPEED);
        frontMotors.setAll(1);
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
            m.set(RobotMap.RAISED_FRONT_INTAKE);
        }
    }

    public void frontIntakeSolenoidOff() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(RobotMap.LOWERED_FRONT_INTAKE);
        }
    }
    /* Rear Solenoids */

    public void rearIntakeSolenoidOn() {
        for (DoubleSolenoid m : rearSolenoids) {
            m.set(RobotMap.RAISED_REAR_INTAKE);
        }
    }

    public void rearIntakeSolenoidOff() {
        for (DoubleSolenoid m : rearSolenoids) {
            m.set(RobotMap.LOWERED_REAR_INTAKE);
        }
    }

    public void setServoPosition(int angle) {
        sensorServo.setAngle(angle);
    }

    public double getServoPosition() {
        return sensorServo.getAngle();
    }

    public enum Color {RED, GREEN, BLUE, YELLOW}

    public static int getIDFromColor(Color color) {

        return controlPanelColors.get(color);
    }

    public static Color getColorFromID(int id) {
        return controlPanelIDs.get(id);
    }

    public TecbotMotorList getFrontMotors() {
        return frontMotors;
    }

    public TecbotMotorList getRearMotors() {
        return rearMotors;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
