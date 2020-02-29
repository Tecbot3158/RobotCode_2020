/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystemCommands.climber.ClimberDisengageGearClimbingMode;
import frc.robot.resources.TecbotConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DisengageLosenRopeAndActivatePulleyMotors extends SequentialCommandGroup {
    /**
     * Creates a new Sequential_CommandGroup.
     */
    public DisengageLosenRopeAndActivatePulleyMotors() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
                //TODO uncomment these commands
                new CancelWinchCommand(),
                new ClimberDisengageGearClimbingMode(),
                new ClimberLosenRopeForSpecificTime().withTimeout(TecbotConstants.WINCH_LOOSEN_ROPE_DEFAULT_TIME),
                //new LosenRopeForSpecificTime(),
                new SetPulleySpeedWithTriggers()
        );
    }
}
