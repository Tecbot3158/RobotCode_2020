/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class ActivateManualWinch extends CommandBase {
    /**
     * Initiates climbing mode and climbs.
     */
    public ActivateManualWinch() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getClimber());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.getRobotContainer().getClimber().getxWhenPressedCount() < 2)
            Robot.getRobotContainer().getClimber().addToXCounter();
        Robot.getRobotContainer().getClimber().resetXCounterAfterTime(2);
        //else Robot.getRobotContainer().getClimber().disengageGear();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        boolean climberLeftLimitSwitchState = Robot.getRobotContainer().getTecbotSensors().getClimberLeftLimitSwitch(),
                climberRightLimitSwitchState = Robot.getRobotContainer().getTecbotSensors().getCLimberRightLimitSwitch();

        double leftJoystickY = OI.getInstance().getCopilot().getLeftAxisY(),
                rightJoystickY = OI.getInstance().getCopilot().getRightAxisY();

        //controlling winch speed
        if (Robot.getRobotContainer().getClimber().getxWhenPressedCount() >= 2) {


            if (!climberLeftLimitSwitchState)
                Robot.getRobotContainer().getClimber().setLeftWinchSpeed(leftJoystickY);
            else Robot.getRobotContainer().getClimber().setLeftWinchSpeed(0);
            if (!climberRightLimitSwitchState)
                Robot.getRobotContainer().getClimber().setRightWinchSpeed(rightJoystickY);
            else Robot.getRobotContainer().getClimber().setRightWinchSpeed(0);

            //climber sensor are independent.

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.getRobotContainer().getClimber().setWinchSpeed(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.getRobotContainer().getClimber().getxWhenPressedCount() < 2 && !Robot.getRobotContainer().getClimber().shrekPowerHasBeenActivated;
    }
}
