/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;

public class ClimberLosenRopeForSpecificTime extends CommandBase {
    /**
     * Creates a new Command.
     */
    public ClimberLosenRopeForSpecificTime() {
        // Use addRequirements() here to declare subsystem dependencies.

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getTransportationSystem());
        addRequirements(Robot.getRobotContainer().getIntake());
        addRequirements(Robot.getRobotContainer().getClimber());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getClimber().setWinchSpeed(TecbotConstants.WINCH_LOOSEN_ROPE_DEFAULT_SPEED, TecbotConstants.WINCH_LOOSEN_ROPE_DEFAULT_SPEED);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.getRobotContainer().getClimber().setWinchSpeed(0, 0);
        System.out.println("setting winch motors to 0");
        System.out.println("-----------------");

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
