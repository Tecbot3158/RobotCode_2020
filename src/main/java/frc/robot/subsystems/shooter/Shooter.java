/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems.shooter;


import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSpeedController;


public class Shooter extends PIDSubsystem {
  List<TecbotSpeedController> shooterLeftMotors;
  List<TecbotSpeedController> shooterRightMotors;
  Servo anglerServo;
  TecbotEncoder shooterEncoder;

  double speed;
  double angle;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(TecbotConstants.K_SHOOTER_P, TecbotConstants.K_SHOOTER_I, TecbotConstants.K_SHOOTER_D));
    
    
    anglerServo = RobotConfigurator.buildServo(RobotMap.ANGLER_PORT);
    shooterEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderLeft() , RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET);


  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    anglerServo.setAngle(angle);
    shoot();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shooterEncoder.getRaw();

  }

  public void shoot() {
    Robot.getRobotContainer().getSharedMotors().setAll(speed, speed);
  }

  public void setShootingSpeed(ShooterPosition position) {

    switch (position) {
    case TRENCH:
      speed = TecbotConstants.TRENCH_SHOOTING_SPEED;
      this.setSetpoint(speed);

      break;

    case TARGET_ZONE:
      speed = TecbotConstants.TARGET_ZONE_SHOOTING_SPEED;
      this.setSetpoint(speed);

      break;

    case INITIATION_LINE:
      speed = TecbotConstants.INITIATION_LINE_SHOOTING_SPEED;
      this.setSetpoint(speed);

      break;

    case OFF:
      speed = TecbotConstants.SHOOTER_OFF;
      this.setSetpoint(speed);

      break;

    default:
      DriverStation.reportError("The set speed isn´t possible", true);
    }
  }

  public void setAnglerDegrees(ShooterPosition position) {
    switch (position) {
    case TRENCH:
      angle = TecbotConstants.TRENCH_SHOOTING_ANGLE;

      break;

    case TARGET_ZONE:

      angle = TecbotConstants.TARGET_ZONE_SHOOTING_ANGLE;

      break;

    case INITIATION_LINE:
      angle = TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;
      break;

    case OFF:
      angle = TecbotConstants.SHOOTER_OFF_ANGLE;
      break;

    default:
      DriverStation.reportError("The set angle isn´t possible", true);

    }

  }

  public void setManualShooter(double manualSpeed) {
    Robot.getRobotContainer().getSharedMotors().setAll(manualSpeed, manualSpeed);
  }

  /**
   * sets the angler manually with the triggers
   * 
   * @param lt Left Trigger
   * @param rt Right Trigger
   */
  public void setManualAngler(double lt, double rt) {
    double manualAngle = -lt + rt;
    anglerServo.set(manualAngle);
  }

  public enum ShooterPosition {
    TRENCH, TARGET_ZONE, INITIATION_LINE, OFF
  }

}