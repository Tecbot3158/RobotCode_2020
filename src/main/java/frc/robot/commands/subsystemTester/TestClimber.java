/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemTester;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.subsystems.climber.ClimberGearsToggle;
import frc.robot.resources.TecbotController;

public class TestClimber extends CommandBase {
    /**
     * Creates a new Command.
     */
    public TestClimber() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getIntake());
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        addRequirements(Robot.getRobotContainer().getClimber());
        addRequirements(Robot.getRobotContainer().getShooter());
        addRequirements(Robot.getRobotContainer().getTransportationSystem());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getShooter().disable();
        CommandScheduler.getInstance().clearButtons();
        OI.getInstance().getPilot().whenPressed(TecbotController.ButtonType.B, new ClimberGearsToggle());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.getRobotContainer().getClimber().setWinchSpeed(OI.getInstance().getPilot().getLeftAxisY(), OI.getInstance().getPilot().getRightAxisY());
        Robot.getRobotContainer().getClimber().setPulleySpeed(OI.getInstance().getPilot().getLeftTrigger(), OI.getInstance().getPilot().getRightTrigger());
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
