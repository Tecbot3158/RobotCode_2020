/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lifter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lifter.Lifter;

public class LifterCommand extends CommandBase {
    /**
     * Creates a new LifterCommand.
     */

    private final Lifter m_lifter;

    public LifterCommand(Lifter lifter) {
        m_lifter = lifter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_lifter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_lifter.liftCommand(0,0);
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
