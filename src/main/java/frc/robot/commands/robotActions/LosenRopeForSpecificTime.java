/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.commands.subsystemCommands.climber.ClimberSetRawWinch;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LosenRopeForSpecificTime extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public LosenRopeForSpecificTime() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                new ClimberSetRawWinch(-RobotMap.CLIMBER_WINCH_SPEED, -RobotMap.CLIMBER_WINCH_SPEED),
                new WaitCommand(3),
                new ClimberSetRawWinch(0, 0)
        );
    }
}
