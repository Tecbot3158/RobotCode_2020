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
  ArrayList<DoubleSolenoid> backSolenoids;

  /**
   * Creates a new Intake.
   */
  public Intake() {

    for (int i = 0; i < RobotMap.frontIntakeMotors.length; i++) {
      frontMotors.add(new TecbotSpeedController(RobotMap.frontIntakeMotors[i], RobotMap.frontIntakeMotorsTypes[i]));
      frontMotors.get(i).setInverted(RobotMap.frontIntakeMotorDirection[i]);
    }

    for (int i = 0; i < RobotMap.backIntakeMotors.length; i++) {
      rearMotors.add(new TecbotSpeedController(RobotMap.backIntakeMotors[i], RobotMap.backIntakeMotorsTypes[i]));
      rearMotors.get(i).setInverted(RobotMap.backIntakeMotorDirection[i]);
    }

    frontSolenoids.add(new DoubleSolenoid(RobotMap.frontsolenoids[0], RobotMap.frontsolenoids[1]));

    backSolenoids.add(new DoubleSolenoid(RobotMap.backsolenoids[2], RobotMap.backsolenoids[3]));
  }

  /* Front Intake */

  public void forwardFrontIntake() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(TecbotConstants.FRONT_INTAKE_SPEED);
    }
  }

  public void offFrontIntake() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(0);
    }
  }

  public void reverseFrontIntake() {
    for (TecbotSpeedController m : frontMotors) {
      m.set(-TecbotConstants.FRONT_INTAKE_SPEED);
    }
  }

  /* Rear Intake */

  public void forwardRearIntake() {
    for (TecbotSpeedController m : rearMotors) {
      m.set(TecbotConstants.REAR_INTAKE_SPEED);
    }
  }

  public void offRearIntake() {
    for (TecbotSpeedController m : rearMotors) {
      m.set(0);
    }
  }

  public void reverseRearIntake() {
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

  public void onFrontSolenoids() {
    for (DoubleSolenoid m : frontSolenoids) {
      m.set(Value.kForward);
    }
  }

  public void offFrontSolenoids() {
    for (DoubleSolenoid m : frontSolenoids) {
      m.set(Value.kReverse);
    }
  }
  /* Rear Solenoids */

  public void onRearSolenoids() {
    for (DoubleSolenoid m : frontSolenoids) {
      m.set(Value.kForward);
    }
  }

  public void offRearSolenoids() {
    for (DoubleSolenoid m : frontSolenoids) {
      m.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
