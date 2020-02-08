/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.shootTransport;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.pctower.TransportationSystemForward;
import frc.robot.commands.subsystems.shooter.ShootFromTrench;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootTrenchAndTransport extends SequentialCommandGroup {
    /**
     * <h3><strong>Shoot from Trench and Transport</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul></li>
     *
     * <li>Power Cell Transportation System
     * <ul><li>forward mode, deflector off</li></ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul><li>On position #INITATION_LINE</li></ul>
     * </li>
     *
     * </ul>
     */
    public ShootTrenchAndTransport() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                new ShootFromTrench(),
                new TransportationSystemForward()
        );
    }
}
