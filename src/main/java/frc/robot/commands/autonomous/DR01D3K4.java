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
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionTurn;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemOff;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemShootingSpeed;
import frc.robot.commands.subsystemCommands.shooter.ShooterOff;

public class DR01D3K4 extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public DR01D3K4() {
        super(  new ShootFromInitiationLineCompensate(),
                new TransportationSystemShootingSpeed(),
                new WaitCommand(3),
                new TransportationSystemOff(),
                new ShooterOff(),
                new SpeedReductionTurn(185, .5),
                new CollectPowerCellsGoBackShoot()
        );
    }
}
