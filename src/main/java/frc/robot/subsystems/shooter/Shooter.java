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
  List <TecbotSpeedController> shooterLeftMotors; 
  List <TecbotSpeedController> shooterRightMotors;
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
        new PIDController(0, 0, 0));

        
    shooterLeftMotors = new ArrayList<>();
    for(int i = 0; i < RobotMap.SHOOTER_LEFT_MOTOR_PORTS.length; i ++){
      shooterLeftMotors.add(new TecbotSpeedController(RobotMap.SHOOTER_LEFT_MOTOR_PORTS[i], RobotMap.SHOOTER_TYPE_OF_MOTORS[i]));//el valor de i es igualado al número de los puertos en los parámetros, entonces entre las llaves  va aumentando el valor del puerto correspondiendo al avance en los parámetros 
      for (int ports : RobotMap.SHOOTER_LEFT_INVERTED_MOTOR_PORTS ){
        if(ports == RobotMap.SHOOTER_LEFT_MOTOR_PORTS[i])
          shooterLeftMotors.get(i).setInverted(true);
      }
    }
    for (int i = 0; i < RobotMap.SHOOTER_RIGHT_MOTOR_PORTS.length; i++) {
    shooterRightMotors.add(new TecbotSpeedController(RobotMap.SHOOTER_RIGHT_MOTOR_PORTS[i], RobotMap.SHOOTER_TYPE_OF_MOTORS[i]));
    for (int ports : RobotMap.SHOOTER_RIGHT_INVERTED_MOTOR_PORTS ){
      if(ports == RobotMap.SHOOTER_RIGHT_MOTOR_PORTS[i])
        shooterRightMotors.get(i).setInverted(true);
  }

    anglerServo = new Servo(RobotMap.ANGLERPORT);
    shooterEncoder = new TecbotEncoder(RobotMap.SHOOTERENCODER_PORT[0], RobotMap.SHOOTERENCODER_PORT[1]);
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
    for(TecbotSpeedController leftmotors : shooterLeftMotors){
      leftmotors.set(speed);
    } 
      for(TecbotSpeedController rightmotors : shooterRightMotors) {
        rightmotors.set(speed);
      }

    anglerServo.set(angle);
  }
  

  public void setShootingSpeed(ShooterPosition position){
    
    switch (position){
      case TRENCH : 
        speed = TecbotConstants.TRENCH_SHOOTING_SPEED ;
        
        break;
      
      case TARGET_ZONE:
        speed = TecbotConstants.TARGET_ZONE_SHOOTING_SPEED; 

        break;
      
      case INITIATION_LINE: 
        speed = TecbotConstants.INITIATION_LINE_SHOOTING_SPEED;
        
        break;

      case OFF: 
        speed = TecbotConstants.SHOOTER_OFF;

        break;
      
      default :
      DriverStation.reportError("The set speed isn´t possible", true);    }


  
    
  }
public void setAngler() {
  anglerServo.setAngle(angle);
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

public void setManualShooter(double manualspeed) {
  for(TecbotSpeedController leftmanualmotors : shooterLeftMotors){
    leftmanualmotors.set(manualspeed);
  }
  for(TecbotSpeedController rightmanualmotors : shooterRightMotors) {
    rightmanualmotors.set(manualspeed);
    SmartDashboard.putNumber("ShooterEncoderRate", shooterEncoder.getRate());

  }
}

/**
 * sets the angler manually with the triggers
 * @param lt Left Trigger
 * @param rt Right Trigger
 */
public void setManualAngler(double lt, double rt){
  double manualangle = -lt + rt;
  anglerServo.set(manualangle);
}

  public enum ShooterPosition{
    TRENCH, 
    TARGET_ZONE,
    INITIATION_LINE, 
    OFF
  }
  
  
  
}