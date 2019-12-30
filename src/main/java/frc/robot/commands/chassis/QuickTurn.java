/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;
import frc.robot.subsystems.chassis.DriveTrain.DrivingMode;

public class QuickTurn extends Command {


    boolean onTarget = false;
    double initialAngle;
    double targetAngle;

    //Saves the wheel solenoid state before the beginning of the command in order to leave it like that at the end of it.
    boolean initialWheelState;

    //Saves the driving mode before the beginning of the command in order to leave it like that at the end of it.
    DrivingMode initialMode;

    public QuickTurn() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        initialMode = Robot.driveTrain.getCurrentDrivingMode();
        initialWheelState = Robot.driveTrain.getWheelState();

        Robot.driveTrain.setWheelState(false);
        onTarget = false;
        initialAngle = Robot.tecbotgyro.getYaw();
        if(initialAngle >=0)
            targetAngle = initialAngle -180;
        else
            targetAngle = initialAngle +180;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double deltaAngle = (-Robot.tecbotgyro.getYaw() + targetAngle);
        //Prevents robot from turning in the incorrect direction
        if(deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        }
        else if(deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }
        Robot.driveTrain.tankDrive(-deltaAngle * TecbotConstants.QUICK_TURN_CORRECTION, deltaAngle * TecbotConstants.QUICK_TURN_CORRECTION);
        if(Math.abs(deltaAngle) <= TecbotConstants.QUICK_TURN_OFFSET){
            onTarget = true;
        }
        System.out.println(deltaAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return onTarget;
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        Robot.driveTrain.setWheelState(initialWheelState);
        Robot.driveTrain.setDrivingMode(initialMode);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
