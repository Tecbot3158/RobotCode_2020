/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromInitiationLineCompensate;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionStraight;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionTurn;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeIn;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeSetRaw;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemForward;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemSetRaw;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemShootingSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CollectPowerCellsGoBackShoot extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public CollectPowerCellsGoBackShoot() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                new FrontIntakeSetRaw(.75),
                new TransportationSystemSetRaw(.4),
                new SpeedReductionStraight(6.2,.4,180),
                new SpeedReductionStraight(-4.9, .8,180),
                new SpeedReductionTurn(-40,.5),
                new WaitCommand(.5),
                new ShootFromInitiationLineCompensate(),
                new TransportationSystemShootingSpeed()
        );
    }
}
