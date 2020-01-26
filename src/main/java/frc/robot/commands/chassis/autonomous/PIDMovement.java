/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;

import static frc.robot.resources.TecbotSensors.SubsystemType.RIGHT_CHASSIS;

public class PIDMovement extends PIDCommand {
    /**
     * Creates a new PID_Command.
     */
    public PIDMovement(double meters) {
        super(
                // The controller that the command will use
                new PIDController(TecbotConstants.K_STRAIGHT_P, TecbotConstants.K_STRAIGHT_I, TecbotConstants.K_STRAIGHT_D),
                // This should return the measurement
                () -> TecbotSensors.getEncoderRaw(RIGHT_CHASSIS),
                // This should return the setpoint (can also be a constant)
                () -> RobotContainer.getDriveTrain().getPIDTarget(),
                // This uses the output
                output -> RobotContainer.driveTrain.useOutput(output)
        );

        RobotContainer.getDriveTrain().setPIDTarget(50);

    }

    public void useOutput(double output){

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
