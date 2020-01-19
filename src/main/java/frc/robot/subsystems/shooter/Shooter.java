/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSpeedController;

public class Shooter extends PIDSubsystem {
  List <TecbotSpeedController> shooterMotors;

  boolean loadingBayShoot = false;
  boolean trenchShoot = false;
  boolean initiationLineShoot = false;
  double speed;


  /**
   * Creates a new Shooter.
   */
  public Shooter() { 
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        
    shooterMotors = new ArrayList<>();
    for(int i = 0; i < RobotMap.SHOOTER_MOTOR_PORTS.length; i ++ ){
      shooterMotors.add(new TecbotSpeedController(RobotMap.SHOOTER_MOTOR_PORTS[i], RobotMap.SHOOTER_TYPE_OF_MOTORS[i]));//el valor de i es igualado al número de los puertos en los parámetros, entonces entre las llaves  va aumentando el valor del puerto correspondiendo al avance en los parámetros 
    }
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
    for(TecbotSpeedController motor : shooterMotors){
      motor.set(speed);
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
        speed = 0;
    }
  
    
  }

  public enum ShooterPosition{
    TRENCH, 
    LOADING_BAY,
    INITIATION_LINE
  }
  
  
}
