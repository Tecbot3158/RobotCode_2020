/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.resources.Math;
import frc.robot.subsystems.chassis.DriveTrain;

public class DefaultDrive extends CommandBase {
    /**
     * Creates a new Command.
     */
    public DefaultDrive() {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // left y
        double y = Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY()), -0.1, 0.1);
        // left x
        double x = Math.clamp((OI.getInstance().getPilot().getLeftAxisX()), -0.1, 0.1);
        if (Robot.getRobotContainer().getDriveTrain().getCurrentDrivingMode() == DriveTrain.DrivingMode.Default) {
            x = (OI.getInstance().getPilot().getLeftAxisX(true));
        }
        // right x
        double turn = (OI.getInstance().getPilot().getRightAxisX());
        // Triggers
        double middleWheel = OI.getInstance().getPilot().getTriggers();
        SmartDashboard.putNumber("TRIGGERS ", middleWheel);

        Robot.getRobotContainer().getDriveTrain().defaultDrive(x, y, turn, middleWheel);
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
