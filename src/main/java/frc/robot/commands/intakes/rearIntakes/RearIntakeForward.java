/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakes.rearIntakes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RearIntakeForward extends InstantCommand {
    /**
     * Add your docs here.
     */
    public RearIntakeForward() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        Robot.getRobotContainer().getIntake().rearIntakeForward();
        addRequirements(Robot.getRobotContainer().getIntake());
    }


}
