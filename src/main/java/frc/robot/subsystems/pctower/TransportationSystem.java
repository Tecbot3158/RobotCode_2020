/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pctower;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSpeedController;


public class TransportationSystem extends SubsystemBase {
    DoubleSolenoid solenoidDeflector; 
    //Servo servoDeflector;
    List <TecbotSpeedController> transportationSystemMotors;
  /**
   * Creates a new Subsystem.
   */
  public TransportationSystem() {
      solenoidDeflector = new DoubleSolenoid (RobotMap.DEFLECTOR_SOLENOID[0], RobotMap.DEFLECTOR_SOLENOID[1]);
      transportationSystemMotors = new ArrayList<>();
      for (int i = 0; i < RobotMap.TRANSPORTATION_SYSTEM_MOTOR_PORTS.length; i ++){
        transportationSystemMotors.add(new TecbotSpeedController(RobotMap.TRANSPORTATION_SYSTEM_MOTOR_PORTS[i], RobotMap.TRANSPORTATION_SYSTEM_TYPE_OF_MOTORS[i])); 
      //El valor de i es igualado al número de los puertos
      }
  }
  
  public void setRaw(double speed){
    for(TecbotSpeedController motor : transportationSystemMotors){ //for te ayuda a acceder a los elementos por cada uno y no a la lista completa
      motor.set(speed);
    }
  }


  public void forward(){
    for(TecbotSpeedController motor : transportationSystemMotors){ //for te ayuda a acceder a los elementos por cada uno y no a la lista completa
      motor.set(TecbotConstants.TRANSPORTATION_SYSTEM_POWER);
    }
  }

  public void reverse(){
    for(TecbotSpeedController motor : transportationSystemMotors){ //: significa en
      motor.set(-TecbotConstants.TRANSPORTATION_SYSTEM_POWER);
    }
  }  

  public void off(){
    for(TecbotSpeedController motor : transportationSystemMotors){
      motor.set(0);
    }
  }   

  public void closeDeflector(){
    solenoidDeflector.set(Value.kForward);
  }

  public void openDeflector(){
    solenoidDeflector.set(Value.kReverse);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}