/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.subsystems.SharedMotors;

public class Shooter extends PIDSubsystem {
  List <TecbotSpeedController> shooterLeftMotors; 
  List <TecbotSpeedController> shooterRightMotors;
  TecbotSpeedController shooterMotorEncoder;
  Servo anglerServo; 
  
  

  
  double speed;
  double angle;


  /**
   * Creates a new Shooter.
   */
  public Shooter() { 
    super(
        // The PIDController used by the subsystem
        new PIDController(TecbotConstants.K_SHOOTER_P, TecbotConstants.K_SHOOTER_I,
            TecbotConstants.K_SHOOTER_D));
        
        
SharedMotors.initializeSharedMotors();
anglerServo = new Servo(RobotMap.ANGLER_PORT);

  }
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    output = speed;
    setpoint = speed;
  anglerServo.setAngle(angle);
  SharedMotors.setAll(output, output);
   
    

  }
  


  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
   
  }
  public void shoot(){
    for(TecbotSpeedController leftMotors : SharedMotors.leftSharedMotors){
      leftMotors.set(speed);
    } 
    for(TecbotSpeedController rightMotors : SharedMotors.rightSharedMotors) {
     rightMotors.set(speed);
      }

    anglerServo.set(angle);
  }
  

  public void setShootingSpeed(ShooterPosition position){
    
    switch (position){
      case TRENCH : 
        speed = TecbotConstants.TRENCH_SHOOTING_SPEED ;
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
      
      default :
      DriverStation.reportError("The set speed isn´t possible", true);    } 
  }

public void setAnglerDegrees(ShooterPosition position) {
  switch(position){
  case TRENCH : 
    angle = TecbotConstants.TRENCH_SHOOTING_ANGLE;

    break;

  case TARGET_ZONE : 

    angle = TecbotConstants.TARGET_ZONE_SHOOTING_ANGLE;

    break;

  case INITIATION_LINE :
    angle = TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;
    break;

  case OFF:
    angle = TecbotConstants.SHOOTER_OFF_ANGLE;
    break;

    default :
      DriverStation.reportError("The set angle isn´t possible", true);
  
  }

  
}

public void setManualShooter(double manualSpeed) {
  for(TecbotSpeedController leftManualMotors : SharedMotors.leftSharedMotors){
    leftManualMotors.set(manualSpeed);
  }
  for(TecbotSpeedController rightManualMotors : SharedMotors.rightSharedMotors) {
    rightManualMotors.set(manualSpeed);
    

  }
}

/**
 * sets the angler manually with the triggers
 * @param lt Left Trigger
 * @param rt Right Trigger
 */
public void setManualAngler(double lt, double rt){
  double manualAngle = -lt + rt;
  anglerServo.set(manualAngle);
}

  public enum ShooterPosition{
    TRENCH, 
    TARGET_ZONE,
    INITIATION_LINE,
    OFF
  }
  
  
  
  
}