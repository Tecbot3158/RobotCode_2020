/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions.intakeTransport;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeOff;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeSolenoidOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeSolenoidOff;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemCloseDeflector;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemOff;

public class IntakesAndTransportOff extends SequentialCommandGroup {
    /**
     *
     */
    public IntakesAndTransportOff() {
        super(
                new FrontIntakeOff(),
                new RearIntakeOff(),
                new RearIntakeSolenoidOff(),
                new FrontIntakeSolenoidOff(),
                new TransportationSystemOff(),
                new TransportationSystemCloseDeflector()
        );
    }
}
