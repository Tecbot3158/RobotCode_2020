/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.mixed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.intakes.rearIntakes.RearIntakeIn;
import frc.robot.commands.subsystems.intakes.rearIntakes.RearIntakeSolenoidOn;
import frc.robot.commands.subsystems.pctower.TransportationSystemForward;
import frc.robot.commands.subsystems.shooter.ShootFromInitiationLine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RearIntakeShootInititationLineTransport extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public RearIntakeShootInititationLineTransport() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                new RearIntakeSolenoidOn(),
                new RearIntakeIn(),
                new ShootFromInitiationLine(),
                new TransportationSystemForward()
        );
    }
}
