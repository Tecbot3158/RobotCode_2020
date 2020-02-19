/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subsystems.pctower.TransportationSystemOff;
import frc.robot.commands.subsystems.pctower.TransportationSystemReverse;
import frc.robot.resources.TecbotConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CompensateForShooting extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public CompensateForShooting() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                new TransportationSystemReverse(),
                new WaitCommand(TecbotConstants.TRANSPORTATION_SYSTEM_REVERSE_TIME_COMPENSATION_IN_SECONDS),
                new TransportationSystemOff()
        );
    }
}
