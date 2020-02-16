/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.chassis.DriveTrain.DrivingMode;

public class QuickTurn extends CommandBase {
    /**
     * Creates a new Command.
     */


    boolean onTarget = false;
    double initialAngle;
    double targetAngle;

    //Saves the wheel solenoid state before the beginning of the command in order to leave it like that at the end of it.
    DriveTrain.WheelState initialWheelState;

    //Saves the driving mode before the beginning of the command in order to leave it like that at the end of it.
    DrivingMode initialMode;

    public QuickTurn() {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialMode = Robot.getRobotContainer().getDriveTrain().getCurrentDrivingMode();
        initialWheelState = Robot.getRobotContainer().getDriveTrain().getDragonFlyWheelState();

        Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Lowered);
        onTarget = false;
        initialAngle = Robot.getRobotContainer().getTecbotSensors().getYaw();
        if(initialAngle >=0)
            targetAngle = initialAngle -180;
        else
            targetAngle = initialAngle +180;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double deltaAngle = (-Robot.getRobotContainer().getTecbotSensors().getYaw() + targetAngle);
        //Prevents robot from turning in the incorrect direction
        if(deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        }
        else if(deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }
        Robot.getRobotContainer().getDriveTrain().tankDrive(-deltaAngle * TecbotConstants.QUICK_TURN_CORRECTION, deltaAngle * TecbotConstants.QUICK_TURN_CORRECTION);
        if(Math.abs(deltaAngle) <= TecbotConstants.QUICK_TURN_OFFSET){
            onTarget = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(initialWheelState);
        Robot.getRobotContainer().getDriveTrain().setDrivingMode(initialMode);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return onTarget;
    }
}
