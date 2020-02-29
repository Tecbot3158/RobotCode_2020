/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.intakeTransport;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeIn;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeSolenoidRaised;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemForward;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemOpenDeflector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RearIntakeAndTransport extends SequentialCommandGroup {

    /**
     * <h3><strong>TRANSPORT active when Power Cell present in Rear
     * INTAKE</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on intake mode, pneumatics on</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector on</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #OFF</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public RearIntakeAndTransport() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                /*
                FI PNEUMATICS OFF
                FI OFF MODE
                RI PNEUMATICS ON
                RI ON MODE
                PCTS ON, DEF OFF
                PCS OFF
                 */
                new RearIntakeSolenoidRaised(),
                new RearIntakeIn(),
                new TransportationSystemForward(),
                new TransportationSystemOpenDeflector()
        );

    }
}
