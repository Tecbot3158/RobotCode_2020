/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class SetPulleySpeedWithTriggers extends CommandBase {
    /**
     * Creates a new Command.
     */
    public SetPulleySpeedWithTriggers() {
        // Use addRequirements() here to declare subsystem dependencies.

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getTransportationSystem());
        addRequirements(Robot.getRobotContainer().getIntake());
        addRequirements(Robot.getRobotContainer().getClimber());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("PULLEY", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightJoystickYCopilot = OI.getInstance().getCopilot().getRightAxisY();
        Robot.getRobotContainer().getClimber().setPulleySpeed(rightJoystickYCopilot);
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
