/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotMotorList;

public class Climber extends SubsystemBase {
    /**
     * Creates a new Climber.
     */
    TecbotMotorList winchMotors;
    DoubleSolenoid gearDisengager;

    public Climber() {
        winchMotors = RobotConfigurator.buildMotorList(RobotMap.WINCH_PORTS, RobotMap.INVERTED_WINCH_PORTS, RobotMap.WINCH_MOTOR_TYPES);
        gearDisengager = new DoubleSolenoid(RobotMap.GEAR_DISENGAGER_PORTS[0], RobotMap.GEAR_DISENGAGER_PORTS[1]);
    }

    public void gearDisengagerToggle() {
        if (gearDisengager.get() == Value.kForward)
            gearDisengager.set(Value.kReverse);
        else
            gearDisengager.set(Value.kForward);
    }

    /**
     * @param winchPower Requires double for the speed for lifting the hook
     */
    public void setWinchSpeed(double winchPower) {
        winchMotors.setAll(winchPower);
    }

    /**
     * @param pulleyPowerRight Requires double for right pulley speed
     * @param pulleyPowerLeft  Requires double for left pulley speed
     */
    public void setPulleySpeed(double pulleyPowerRight, double pulleyPowerLeft) {
        //reel, cannot do this if input is negative
        RobotContainer.sharedMotors.setAll(Math.abs(pulleyPowerRight), Math.abs(pulleyPowerLeft));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
