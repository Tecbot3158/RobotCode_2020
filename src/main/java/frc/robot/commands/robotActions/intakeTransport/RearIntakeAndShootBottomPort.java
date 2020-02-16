/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.intakeTransport;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.intakes.frontIntakes.FrontIntakeIn;
import frc.robot.commands.subsystems.intakes.frontIntakes.FrontIntakeOut;
import frc.robot.commands.subsystems.intakes.frontIntakes.FrontIntakeSolenoidOff;
import frc.robot.commands.subsystems.intakes.rearIntakes.RearIntakeIn;
import frc.robot.commands.subsystems.intakes.rearIntakes.RearIntakeSolenoidOn;
import frc.robot.commands.subsystems.pctower.TransportationSystemForward;
import frc.robot.commands.subsystems.pctower.TransportationSystemOpenDeflector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RearIntakeAndShootBottomPort extends SequentialCommandGroup {

    /**
     * <h3><strong>Front Intake Shoot Bottom Port and Intake from RearIntake</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on shooter mode, pneumatics off</li>
     * <li>Rear intake on intake mode, pneumatics on</li>
     * </ul></li>
     *
     * <li>Power Cell Transportation System
     * <ul><li>forward mode, deflector on</li></ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul><li>On position #OFF</li></ul>
     * </li>
     *
     * </ul>
     */
    public RearIntakeAndShootBottomPort() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                //new FrontIntakeSolenoidOff(),
                new FrontIntakeOut(),
                new RearIntakeSolenoidOn(),
                new RearIntakeIn(),
                new TransportationSystemForward(),
                new TransportationSystemOpenDeflector()
        );
    }
}