/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.resources.*;

public class Intake extends SubsystemBase {

  ArrayList<TecbotSpeedController> frontMotors;
  ArrayList<TecbotSpeedController> rearMotors;

  ArrayList<DoubleSolenoid> frontSolenoids;
  ArrayList<DoubleSolenoid> rearSolenoids;

  /**
   * Creates a new Intake.
   */
  public Intake() {

    for (int i = 0; i < RobotMap.FRONT_INTAKE_MOTORS.length; i++) {
      frontMotors
          .add(new TecbotSpeedController(RobotMap.FRONT_INTAKE_MOTORS[i], RobotMap.FRONT_INTAKE_MOTORS_TYPES[i]));
      frontMotors.get(i).setInverted(RobotMap.FRONT_INTAKE_MOTOR_DIRECTION[i]);
    }

    for (int i = 0; i < RobotMap.REAR_INTAKE_MOTORS.length; i++) {
      rearMotors.add(new TecbotSpeedController(RobotMap.REAR_INTAKE_MOTORS[i], RobotMap.BACK_INTAKE_MOTORS_TYPES[i]));
      rearMotors.get(i).setInverted(RobotMap.BACK_INTAKE_MOTOR_DIRECTION[i]);
    }

    frontSolenoids.add(new DoubleSolenoid(RobotMap.FRONT_SOLENOIDS[0], RobotMap.FRONT_SOLENOIDS[1]));

    rearSolenoids.add(new DoubleSolenoid(RobotMap.BACK_SOLENOIDS[2], RobotMap.BACK_SOLENOIDS[3]));
  }

  /* Front Intake */

  public void frontIntakeForward() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(TecbotConstants.FRONT_INTAKE_SPEED);
    }
  }

  public void frontIntakeOff() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(0);
    }
  }

  public void frontIntakeReverse() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(-TecbotConstants.FRONT_INTAKE_SPEED);
    }
  }

  /* Rear Intake */

  public void rearIntakeForward() {
    for (TecbotSpeedController m : rearMotors) {
      m.set(TecbotConstants.REAR_INTAKE_SPEED);
    }
  }

  public void rearIntakeOff() {
    for (TecbotSpeedController m : rearMotors) {
      m.set(0);
    }
  }

  public void rearIntakeReverse() {
    for (TecbotSpeedController m : rearMotors) {
      m.set(-TecbotConstants.REAR_INTAKE_SPEED);
    }
  }

  public void setRawFrontIntake(double speed) {
    for (TecbotSpeedController m : frontMotors) {
      m.set(speed);
    }
  }

  public void setRawRearIntake(double speed) {
    for (TecbotSpeedController m : rearMotors) {
      m.set(speed);
    }
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
