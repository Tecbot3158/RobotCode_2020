/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;

public class ActivateShrekPower extends CommandBase {
    /**
     * Initiates climbing mode and climbs.
     */
    public ActivateShrekPower() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getClimber());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.getRobotContainer().getClimber().getxWhenPressedCount() < 2)
            Robot.getRobotContainer().getClimber().addToXCounter();
        else Robot.getRobotContainer().getClimber().disengageGear();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.getRobotContainer().getClimber().getxWhenPressedCount() > 2) {
            if (!Robot.getRobotContainer().getTecbotSensors().getClimberLimitSwitch())
                Robot.getRobotContainer().getClimber().setWinchSpeed(TecbotConstants.WINCH_SPEED, TecbotConstants.WINCH_SPEED);
            else {
                double leftSpeedPulley = OI.getInstance().getPilot().getLeftTrigger();
                double rightSpeedPulley = OI.getInstance().getPilot().getRightTrigger();
                Robot.getRobotContainer().getClimber().setWinchSpeed(-leftSpeedPulley, -rightSpeedPulley);
                Robot.getRobotContainer().getClimber().setPulleySpeed(leftSpeedPulley, rightSpeedPulley);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.getRobotContainer().getClimber().getxWhenPressedCount() < 2;
    }
}
