/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemTester;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.subsystemCommands.climber.ClimberGearsToggle;
import frc.robot.resources.TecbotController;

public class TestRightClimber extends CommandBase {
    /**
     * Creates a new Command.
     */
    public TestRightClimber() {
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
        OI.getInstance().getPilot().clearPOVCommands();
        OI.getInstance().getPilot().whenPressed(TecbotController.ButtonType.B, new ClimberGearsToggle());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.currentMotorBeingTested < Robot.getRobotContainer().getSharedMotors().rightSharedMotors.size()) {
            Robot.getRobotContainer().getSharedMotors().rightSharedMotors.getMotors().get(Robot.currentMotorBeingTested).set(
                    OI.getInstance().getPilot().getLeftAxisY()
            );
        } else {
            Robot.currentMotorBeingTested = 0;
        }

        if (Robot.currentSecondMotorBeingTested < Robot.getRobotContainer().getClimber().getRightWinchMotors().size()) {
            Robot.getRobotContainer().getClimber().getRightWinchMotors().getMotors().get(Robot.currentMotorBeingTested).set(
                    OI.getInstance().getPilot().getTriggers()
            );
        } else {
            Robot.currentSecondMotorBeingTested = 0;
        }
        SmartDashboard.putNumber("rightCLimber", OI.getInstance().getPilot().getTriggers());
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
