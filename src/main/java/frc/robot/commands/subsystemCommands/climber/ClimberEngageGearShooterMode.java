/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ClimberEngageGearShooterMode extends InstantCommand {
    /**
     * Creates a new Command.
     */
    public ClimberEngageGearShooterMode() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getClimber().engageGear();
    }

}
