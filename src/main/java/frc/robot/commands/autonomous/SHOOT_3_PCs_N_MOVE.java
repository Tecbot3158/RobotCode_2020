/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootRawCompensate;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionStraight;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionTurn;
import frc.robot.commands.subsystemCommands.chassis.drivingModes.ChassisSetSpeed;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeSolenoidLowered;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemOff;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemShootingSpeed;
import frc.robot.commands.subsystemCommands.shooter.ShooterOff;
import frc.robot.resources.TecbotConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SHOOT_3_PCs_N_MOVE extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public SHOOT_3_PCs_N_MOVE() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(

                new ChassisSetSpeed(),
                new FrontIntakeSolenoidLowered(),
                //new ShootFromInitiationLineCompensate(),
                new ShootRawCompensate(TecbotConstants.SHOOTER_AUTONOMOUS_SPEED_DR01D3K4),
                new TransportationSystemShootingSpeed(),
                new WaitCommand(3),
                new TransportationSystemOff(),
                new ShooterOff(),
                new SpeedReductionStraight(-2, 0.5, 180)
        );
    }
}
