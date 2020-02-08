/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.intakes.rearIntakes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RearIntakeIn extends InstantCommand {
    /**
     * Add your docs here.
     */
    public RearIntakeIn() {
        super();
        // Use requires() here to declare subsystem dependencies
        addRequirements(Robot.getRobotContainer().getIntake());
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        Robot.getRobotContainer().getIntake().rearIntakeForward();
    }


}
