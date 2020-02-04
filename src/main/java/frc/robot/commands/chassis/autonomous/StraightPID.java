/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StraightPID extends PIDCommand {
    /**
     * Creates a new PID_Command.
     */
    public StraightPID() {
        super(
                // The controller that the command will use
                new PIDController(TecbotConstants.K_STRAIGHT_P, TecbotConstants.K_STRAIGHT_I, TecbotConstants.K_STRAIGHT_D),
                // This should return the measurement
                () -> TecbotSensors.getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS),
                // This should return the setpoint (can also be a constant)
                () -> Robot.m_robotContainer.getDriveTrain().getPidStraightTarget(),
                // This uses the output
                output -> {
                    // Use the output here
                    Robot.m_robotContainer.getDriveTrain().moveStraightPID(output);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(TecbotSensors.getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS) - Robot.m_robotContainer.getDriveTrain().getPidStraightTarget()) < TecbotConstants.K_PID_STRAIGHT_ARRIVE_OFFSET;
    }
}
