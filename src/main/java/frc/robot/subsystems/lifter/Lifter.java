/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.lifter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.subsystems.SharedMotors;

public class Lifter extends SubsystemBase {
    /**
     * Creates a new Lifter.
     */
    List<TecbotSpeedController> winchMotors;
    DoubleSolenoid gearShifter;
    int encoderMotor = -1;

    public Lifter() {
        winchMotors = new ArrayList<>();
        for (int i = 0; i < RobotMap.winchPorts.length; i++) {
            winchMotors.add(new TecbotSpeedController(RobotMap.winchPorts[i], RobotMap.winchTypesOfMotors[i]));
            if (RobotMap.winchPorts[i] == RobotMap.invertedWinchMotors[i]) {
                winchMotors.get(i).setInverted(true);
            }
        }
        gearShifter = new DoubleSolenoid(RobotMap.gearShifterPneumatics[0], RobotMap.gearShifterPneumatics[1]);
    }

    public void disengageGearsToggle() {
        if (gearShifter.get() == Value.kForward)
            gearShifter.set(Value.kReverse);
        else
            gearShifter.set(Value.kForward);
    }

    public void manualLifter(double rightWinch, double leftWinch, double rightPulley, double leftPulley) {
        SharedMotors.setAll(rightPulley, leftPulley);
        winchMotors.get(0).set(rightWinch);
        winchMotors.get(1).set(leftWinch);
    }

    public void liftCommand(double winchPower, double pulleyPowerRight, double pulleyPowerLeft) {
        //lift hook
        for (TecbotSpeedController motor : winchMotors) {
            motor.set(winchPower);
        }
        //reel, cannot do this if input is negative
        if (pulleyPowerRight >= 0 && pulleyPowerLeft >= 0) {
            SharedMotors.setAll(pulleyPowerRight, pulleyPowerLeft);
        }
    }

    public TecbotSpeedController getMotorWithEncoder() {
        return (encoderMotor > 0) ? winchMotors.get(encoderMotor) : null;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
