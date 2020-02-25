/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotActionsCatalog;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromInitiationLineCompensate;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromTrenchCompensate;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionStraight;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionTurn;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemShootingSpeed;
import frc.robot.commands.subsystemCommands.shooter.ShootFromInitiationLine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DR01D3K4 extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public DR01D3K4() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(//new ShootFromInitiationLineCompensate(),
                //new WaitCommand(4),
                //new TransportationSystemShootingSpeed(),
                new SpeedReductionTurn(180, .5)
                //RobotActionsCatalog.getInstance().getRearIntakeAndTransport(),
                , new WaitCommand(1),
                new SpeedReductionStraight(3.4,.5,180),
                new WaitCommand(1),
                //RobotActionsCatalog.getInstance().getIntakesAndTransportOff(),
                new SpeedReductionStraight(-2.4,.5,180),
                new WaitCommand(1),
                new SpeedReductionTurn(-15,.5)
                //new ShootFromTrenchCompensate(),
                //new WaitCommand(4),
                //new TransportationSystemShootingSpeed()
        );
    }
}
