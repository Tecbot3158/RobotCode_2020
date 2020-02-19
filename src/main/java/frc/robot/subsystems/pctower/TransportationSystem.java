/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pctower;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotMotorList;

import java.util.List;


public class TransportationSystem extends SubsystemBase {
    DoubleSolenoid solenoidDeflector; 
    //Servo servoDeflector;
    TecbotMotorList transportationSystemMotors;
  /**
   * Creates a new Subsystem.
   */
  public TransportationSystem() {
      solenoidDeflector = RobotConfigurator.buildDoubleSolenoid(RobotMap.DEFLECTOR_SOLENOID_PORTS);
      transportationSystemMotors = RobotConfigurator.buildMotorList(RobotMap.TRANSPORTATION_SYSTEM_MOTOR_PORTS, RobotMap.TRANSPORTATION_SYSTEM_INVERTED_MOTOR_PORTS, RobotMap.TRANSPORTATION_SYSTEM_TYPE_OF_MOTORS);
  }
  
  public void setRaw(double speed){
    transportationSystemMotors.setAll(speed);
  }


  public void forward(){
    transportationSystemMotors.setAll(TecbotConstants.TRANSPORTATION_SYSTEM_POWER);
  }

  public void reverse(){
    transportationSystemMotors.setAll(-TecbotConstants.TRANSPORTATION_SYSTEM_POWER);
  }

  public void off(){
    transportationSystemMotors.setAll(0);
  }   

  public void closeDeflector(){
    solenoidDeflector.set(Value.kForward);
  }

  public void openDeflector(){
    solenoidDeflector.set(Value.kReverse);
}

  public TecbotMotorList getMotors(){
    return transportationSystemMotors;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}