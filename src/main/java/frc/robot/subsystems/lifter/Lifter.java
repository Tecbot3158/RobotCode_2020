/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.lifter;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

public class Lifter extends SubsystemBase {
    /**
     * Creates a new Lifter.
     */
    List<TecbotSpeedController> winchMotors;
    List<TecbotSpeedController> pulleyMotors;
    DoubleSolenoid gearShifter;
    int encoderMotor = -1;

    public Lifter() {
        for (int i = 0; i < RobotMap.winchPorts.length; i++) {
            winchMotors.add(new TecbotSpeedController(RobotMap.winchPorts[i], RobotMap.typesOfMotors[0]));
            if (RobotMap.winchPorts[i] == RobotMap.invertedWinchMotors[i]) {
                winchMotors.get(i).setInverted(true);
            }
        }
        for (int i = 0; i < RobotMap.pulleyPorts.length; i++) {
            pulleyMotors.add(new TecbotSpeedController(RobotMap.pulleyPorts[i], RobotMap.typesOfMotors[0]));
            if (RobotMap.pulleyPorts[i] == RobotMap.invertedPulleyMotors[i]) {
                pulleyMotors.get(i).setInverted(true);
            }
        }
        DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.gearShifterPneumatics[0],
                RobotMap.gearShifterPneumatics[1]);
    }
    public void shiftGearsToggle() {
        if (gearShifter.get() == Value.kForward)
            gearShifter.set(Value.kReverse);
        else
            gearShifter.set(Value.kForward);
    }

    public void manualLifter(double rightWinch, double leftWinch, double rightPulley, double leftPulley) {
        winchMotors.get(0).set(rightWinch);
        winchMotors.get(1).set(rightWinch);
        winchMotors.get(2).set(leftWinch);
        winchMotors.get(3).set(leftWinch);
        pulleyMotors.get(0).set(rightPulley);
        pulleyMotors.get(1).set(leftPulley);
    }

    /**
     *
     * @param winchPower power for lifting hook
     * @param pulleyPower power for reeling
     */
    public void liftCommand(double winchPower, double pulleyPower) {
        //lift
        if (winchPower != 0) {
            for (TecbotSpeedController motor : pulleyMotors) {
                motor.set(pulleyPower);
            }
        }
        //reel
        for (TecbotSpeedController motor : winchMotors) {
            motor.set(winchPower);
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
