/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.shootTransport;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemSetRaw;
import frc.robot.commands.subsystemCommands.shooter.ShootRaw;

public class NoPIDShootTrenchAndTransport extends SequentialCommandGroup {
    /**
     * Creates a new Command.
     */
    public NoPIDShootTrenchAndTransport() {
        // Use addRequirements() here to declare subsystem dependencies.
            super(new TransportationSystemSetRaw(-.5),new ShootRaw(.9),new WaitCommand(0.5),
                    new TransportationSystemSetRaw(1));
    }

}
