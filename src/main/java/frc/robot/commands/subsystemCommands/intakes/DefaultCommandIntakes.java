/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.intakes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotController;
import frc.robot.subsystems.intake.Intake;

public class DefaultCommandIntakes extends CommandBase {
    /**
     * Creates a new Command.
     */
    Intake intakes;
    TecbotController pilot, copilot;

    public DefaultCommandIntakes() {
        // Use addRequirements() here to declare subsystem dependencies.
        intakes = Robot.getRobotContainer().getIntake();
        pilot = OI.getInstance().getPilot();
        copilot = OI.getInstance().getCopilot();
        addRequirements(intakes);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double frontSpeed, rearSpeed, copilotRightAxisY = copilot.getRightAxisY();

        frontSpeed = copilotRightAxisY;
        //if (intakes.getRearIntakeSolenoidState() == RobotMap.REAR_INTAKE_LOWERED_SOLENOID_VALUE)
            //    rearSpeed = copilotRightAxisY;

        intakes.setRawFrontIntake(frontSpeed);
        //intakes.setRawRearIntake(rearSpeed);

        //if(intakes.getRearIntakeSolenoidState() == DoubleSolenoid.Value.kReverse)
        //  intakes.

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
