/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.wheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class RiseWheel extends InstantCommand {

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(true);
        Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Default);
    }
}
