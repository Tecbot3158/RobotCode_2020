/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;

import static frc.robot.resources.TecbotConstants.K_TURN_D;
import static frc.robot.resources.TecbotConstants.K_TURN_I;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDTurn extends PIDCommand {
    /**
     * Creates a new PID_Command.
     */

    public PIDTurn() {
        super(
                // The controller that the command will use
                new PIDController(TecbotConstants.K_TURN_P, K_TURN_I, K_TURN_D),
                // This should return the measurement
                () -> Robot.getRobotContainer().getTecbotSensors().getYaw(),
                // This should return the setpoint (can also be a constant)
                () -> Robot.getRobotContainer().getDriveTrain().getPidAngleTarget(),
                // This uses the output
                output -> {
                    // Use the output here
                    Robot.getRobotContainer().getDriveTrain().pidTurn(output);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(Robot.getRobotContainer().getDriveTrain().getPidAngleTarget() - Robot.getRobotContainer().getTecbotSensors().getYaw()) <= TecbotConstants.K_PID_STRAIGHT_ARRIVE_OFFSET);
    }
}
