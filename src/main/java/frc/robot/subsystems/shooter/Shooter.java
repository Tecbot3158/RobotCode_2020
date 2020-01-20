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
import frc.robot.resources.TecbotSpeedController;

public class Shooter extends PIDSubsystem {
  List <TecbotSpeedController> shooterleftMotors; 
  List <TecbotSpeedController> shooterrightMotors;
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

        
    shooterleftMotors = new ArrayList<>();
    for(int i = 0; i < RobotMap.SHOOTER_LEFT_MOTOR_PORTS.length; i ++){
      shooterleftMotors.add(new TecbotSpeedController(RobotMap.SHOOTER_LEFT_MOTOR_PORTS[i], RobotMap.SHOOTER_TYPE_OF_MOTORS[i]));//el valor de i es igualado al número de los puertos en los parámetros, entonces entre las llaves  va aumentando el valor del puerto correspondiendo al avance en los parámetros 
    }
    for (int i = 0; i < RobotMap.SHOOTER_RIGHT_MOTOR_PORTS.length; i++) {
    shooterrightMotors.add(new TecbotSpeedController(RobotMap.SHOOTER_RIGHT_MOTOR_PORTS[i], RobotMap.SHOOTER_TYPE_OF_MOTORS[i]));
    shooterrightMotors.get(i).setInverted(true);
  }

    Angler = new Servo(RobotMap.ANGLERPORT);
    shooterEncoder = new TecbotEncoder(RobotMap.SHOOTERENCODER_PORT[0], RobotMap.SHOOTERENCODER_PORT[1]);
  
  }
    
  

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
  public void shoot(){
    for(TecbotSpeedController leftmotors : shooterleftMotors){
      leftmotors.set(speed);
      for(TecbotSpeedController rightmotors : shooterrightMotors) {
        rightmotors.set(speed);
      }
      shooterEncoder.getRate();
    }
  }


  

  public void setShootingSpeed(ShooterPosition position){
    
    switch (position){
      case TRENCH : 
        speed = TecbotConstants.TRENCH_SHOOTING_SPEED ;
        
        break;
      
      case LOADING_BAY:
        speed = TecbotConstants.LOADING_BAY_SHOOTING_SPEED;

        break;
      
      case INITIATION_LINE: 
        speed = TecbotConstants.INITIATION_LINE_SHOOTING_SPEED;
      
      default :
      DriverStation.reportError("The set speed isn´t possible", true);    }
  
    
  }
public void setAngler() {
  Angler.setAngle(angle);
}

public void setAnglerDegrees(ShooterPosition position) {
  switch(position){
  case TRENCH : 
  angle = TecbotConstants.TRENCH_SHOOTING_ANGLE;

  break;

  case LOADING_BAY : 

  angle = TecbotConstants.LOADING_BAY_SHOOTING_ANGLE;

  break;

  case INITIATION_LINE :
  angle = TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;

    default :
      DriverStation.reportError("The set angle isn´t possible", true);
  
  }

  
}

public void setManualShooter(double manualspeed) {
  for(TecbotSpeedController leftmanualmotors : shooterleftMotors){
    leftmanualmotors.set(manualspeed);
  }
  for(TecbotSpeedController rightmanualmotors : shooterrightMotors) {
    rightmanualmotors.set(manualspeed);
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

public void setshooterEncoder() {
  SmartDashboard.putNumber("ShooterEncoderRate", shooterEncoder.getRate());
}
  public enum ShooterPosition{
    TRENCH, 
    LOADING_BAY,
    INITIATION_LINE
  }
  
  
  
}