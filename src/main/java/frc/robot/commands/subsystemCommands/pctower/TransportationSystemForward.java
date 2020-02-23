/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.pctower;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * An example command that uses an example subsystem.
 */
public class TransportationSystemForward extends InstantCommand {

    public TransportationSystemForward() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getTransportationSystem().forward();
    }
}