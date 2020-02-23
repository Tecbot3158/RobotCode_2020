/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.Climber;

public class ClimberSetRawWinch extends CommandBase {
    /**
     * Creates a new ClimberCommand.
     */

    private final Climber m_climber = Robot.getRobotContainer().getClimber();
    private double leftSpeed, rightSpeed;

    public ClimberSetRawWinch(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climber.setWinchSpeed(leftSpeed, rightSpeed);
    }
}
