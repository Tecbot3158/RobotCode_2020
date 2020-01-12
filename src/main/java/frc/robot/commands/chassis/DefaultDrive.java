/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DefaultDrive extends CommandBase {
    /**
     * Creates a new Command.
     */
    public DefaultDrive() {
        addRequirements(RobotContainer.getDriveTrain());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // left y
        double y = -(RobotContainer.oi.getPilot().getLeftAxisY() );
        // left x
        double x = (RobotContainer.oi.getPilot().getLeftAxisX() );
        // right x
        double turn = (RobotContainer.oi.getPilot().getRightAxisX());
        // Triggers
        double middleWheel = RobotContainer.oi.getPilot().getTriggers();

        RobotContainer.getDriveTrain().defaultDrive(x,y,turn,middleWheel);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
