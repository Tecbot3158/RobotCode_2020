/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.Climber;

public class ClimberPulleyCommand extends CommandBase {
    /**
     * Creates a new ClimberPulleyCommand.
     */
    private final Climber m_climber = Robot.getRobotContainer().getClimber();
    private double pulleyPowerRight, pulleyPowerLeft;

    public ClimberPulleyCommand(double powerRight, double powerLeft) {
        pulleyPowerRight = powerRight;
        pulleyPowerLeft = powerLeft;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climber.setPulleySpeed(pulleyPowerRight, pulleyPowerLeft);
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
