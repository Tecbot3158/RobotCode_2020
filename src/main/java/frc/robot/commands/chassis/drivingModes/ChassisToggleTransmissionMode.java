/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.drivingModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class ChassisToggleTransmissionMode extends InstantCommand {
    public ChassisToggleTransmissionMode() {
    }

    @Override
    public void initialize() {
        if(Robot.getRobotContainer().getDriveTrain().getTransmissionMode() == DriveTrain.TransmissionMode.speed)
            Robot.getRobotContainer().getDriveTrain().setTransmissionState(DriveTrain.TransmissionMode.torque);
        else
            Robot.getRobotContainer().getDriveTrain().setTransmissionState(DriveTrain.TransmissionMode.speed);
    }
}
