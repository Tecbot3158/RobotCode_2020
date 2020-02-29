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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.resources.*;

public class Intake extends SubsystemBase {

    TecbotMotorList frontMotors;
    TecbotMotorList rearMotors;

    ArrayList<DoubleSolenoid> frontSolenoids;
    private ArrayList<DoubleSolenoid> rearSolenoids;

    static HashMap<Color, Integer> controlPanelColors;
    static HashMap<Integer, Color> controlPanelIDs;

    double rearSpeed = 0, frontSpeed = 0;

    Servo sensorServo;

    /**
     * Creates a new Intake.
     */
    public Intake() {

        rearSolenoids = new ArrayList<>();
        frontSolenoids = new ArrayList<>();

        frontMotors = RobotConfigurator.buildMotorList(RobotMap.FRONT_INTAKE_MOTOR_PORTS, RobotMap.FRONT_INTAKE_INVERTED_MOTOR_PORTS, RobotMap.FRONT_INTAKE_MOTOR_TYPES);

        //rearMotors = RobotConfigurator.buildMotorList(RobotMap.REAR_INTAKE_MOTOR_PORTS, RobotMap.REAR_INTAKE_INVERTED_MOTOR_PORTS, RobotMap.REAR_INTAKE_MOTOR_TYPES);

        frontSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.FRONT_INTAKE_SOLENOID_PORTS));

        //rearSolenoids.add(RobotConfigurator.buildDoubleSolenoid(RobotMap.REAR_INTAKE_SOLENOID_PORTS));

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
        frontSpeed = TecbotConstants.FRONT_INTAKE_SPEED;
        frontMotors.setAll(frontSpeed);
        //frontMotors.setAll(1);
    }

    public void frontIntakeOff() {
        frontSpeed = 0;
        frontMotors.setAll(frontSpeed);
    }

    public void frontIntakeReverse() {
        frontSpeed = -TecbotConstants.FRONT_INTAKE_SPEED;
        frontMotors.setAll(frontSpeed);
    }

    public void setRawFrontIntake(double speed) {
        frontSpeed = speed;
        frontMotors.setAll(frontSpeed);
    }

    /* Rear Intake */

    public void rearIntakeForward() {
        rearSpeed = TecbotConstants.REAR_INTAKE_SPEED;
        //rearMotors.setAll(rearSpeed);
    }

    public void rearIntakeOff() {
        rearSpeed = 0;
        //rearMotors.setAll(rearSpeed);
    }

    public void rearIntakeReverse() {
        rearSpeed = -TecbotConstants.REAR_INTAKE_SPEED;
        //rearMotors.setAll(rearSpeed);
    }

    public void setRawRearIntake(double speed) {
        rearSpeed = speed;
        //rearMotors.setAll(rearSpeed);
    }

    /* Front Solenoids */

    public void frontIntakeToggleSolenoid() {
        if (frontSolenoids.get(0).get() == RobotMap.FRONT_INTAKE_RAISED_SOLENOID_VALUE) {
            for (DoubleSolenoid m : frontSolenoids) {
                m.set(RobotMap.FRONT_INTAKE_LOWERED_SOLENOID_VALUE);
            }
        } else {
            for (DoubleSolenoid m : frontSolenoids) {
                m.set(RobotMap.FRONT_INTAKE_RAISED_SOLENOID_VALUE);
            }
        }
    }

    public void rearIntakeToggleSolenoid() {
        if (rearSolenoids.get(0).get() == RobotMap.REAR_INTAKE_RAISED_SOLENOID_VALUE) {
            for (DoubleSolenoid m : rearSolenoids) {
                m.set(RobotMap.REAR_INTAKE_LOWERED_SOLENOID_VALUE);
            }
        } else {
            for (DoubleSolenoid m : rearSolenoids) {
                m.set(RobotMap.REAR_INTAKE_RAISED_SOLENOID_VALUE);
            }
        }
    }

    public void frontIntakeSolenoidRaised() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(RobotMap.FRONT_INTAKE_RAISED_SOLENOID_VALUE);
        }
    }

    public void frontIntakeSolenoidLowered() {
        for (DoubleSolenoid m : frontSolenoids) {
            m.set(RobotMap.FRONT_INTAKE_LOWERED_SOLENOID_VALUE);
        }
    }
    /* Rear Solenoids */

    public void rearIntakeSolenoidRaised() {
        for (DoubleSolenoid m : rearSolenoids) {
            m.set(RobotMap.REAR_INTAKE_RAISED_SOLENOID_VALUE);
        }
    }

    public void rearIntakeSolenoidLowered() {
        for (DoubleSolenoid m : rearSolenoids) {
            m.set(RobotMap.REAR_INTAKE_LOWERED_SOLENOID_VALUE);
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

    public Value getFrontIntakeSolenoidState() {
        return frontSolenoids.get(0).get();
    }

    public Value getRearIntakeSolenoidState() {
        //return rearSoleno ids.get(0).get();
        return Value.kForward;
    }

    public void debug(boolean frontIntake, boolean rearIntake) {

        if (frontIntake) {
            SmartDashboard.putBoolean("¬LOW FI", getFrontIntakeSolenoidState() == RobotMap.FRONT_INTAKE_LOWERED_SOLENOID_VALUE);
            SmartDashboard.putNumber("~SPD FI", frontSpeed);

        }
        if (rearIntake) {
            SmartDashboard.putBoolean("LOW RI¬", getFrontIntakeSolenoidState() == RobotMap.FRONT_INTAKE_LOWERED_SOLENOID_VALUE);
            SmartDashboard.putNumber("SPD RI~", rearSpeed);
        }
    }
}