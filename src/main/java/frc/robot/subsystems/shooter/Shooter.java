/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSharedMotors;
import frc.robot.resources.TecbotSpeedController;
import jdk.nashorn.api.tree.ReturnTree;

public class Shooter extends PIDSubsystem {


  Servo Angler;


  TecbotEncoder shooterEncoder;

  boolean loadingBayShoot = false;
  boolean trenchShoot = false;
  boolean initiationLineShoot = false;
  double speed;
  double angle;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    TecbotSharedMotors.initializeSharedMotors();


  }



  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shooterEncoder.getRaw();
  }


  /**
   * Returns the Shooter_PID_Target
   */
  

  public void useOutput(double output) {
    
    TecbotSharedMotors.setAll(output, output);
  }

  
  


  

  public double getShootingSpeed(ShooterPosition position){
    
    switch (position){
      case TRENCH : 
        return TecbotConstants.TRENCH_SHOOTING_SPEED ;
        
        
      
      case LOADING_BAY:
        return TecbotConstants.LOADING_BAY_SHOOTING_SPEED;

        
      
      case INITIATION_LINE: 
        return  TecbotConstants.INITIATION_LINE_SHOOTING_SPEED;
      
      default :
        return 0;
    }
  
    
  }
public void setAngler() {
  Angler.setAngle(angle);
}

public double getAnglerDegrees(ShooterPosition position) {
  switch(position){
  case TRENCH : 
  return TecbotConstants.TRENCH_SHOOTING_ANGLE;

  

  case LOADING_BAY : 

  return TecbotConstants.LOADING_BAY_SHOOTING_ANGLE;

 

  case INITIATION_LINE :

  angle = TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;

  
  return TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;

  default: 
  
  return 0;
  
  }

  
}


/**
 * sets the angler manually with the triggers
 * @param lt Left Trigger
 * @param rt Right Trigger
 */
public void setManualAngler(double lt, double rt){
  double manualangle = -lt + rt;
  Angler.set(manualangle);
}


  
public enum ShooterPosition{

  
    TRENCH, 
    LOADING_BAY,
    INITIATION_LINE
  }
  
  
  
  
}