/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeOff;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeSolenoidOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeSolenoidOff;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemCloseDeflector;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemOff;
import frc.robot.commands.subsystemCommands.shooter.ShootManual;
import frc.robot.commands.subsystemCommands.shooter.ShooterOff;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AllSystemsOff extends SequentialCommandGroup {
    /**
     * <h2><strong>THIS COMMAND SHOULD BE CALLED FOR EVERY
     * whenReleased() used button.</strong></h2>
     * <h3><strong>ALL SYSTEMS OFF</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Off mode, deflector off</li>
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
    public AllSystemsOff() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                /*
                FI PNEUMATICS OFF
                FI OFF MODE
                RI PNEUMATICS OFF
                RI OFF MODE
                PCTS OFF, DEF OFF
                PCS OFF
                 */
                new FrontIntakeSolenoidOff(),
                new FrontIntakeOff(),
                new RearIntakeSolenoidOff(),
                new RearIntakeOff(),
                new TransportationSystemOff(),
                new TransportationSystemCloseDeflector(),
                new ShooterOff(),
                new ShootManual(0,0)
        );
    }
}