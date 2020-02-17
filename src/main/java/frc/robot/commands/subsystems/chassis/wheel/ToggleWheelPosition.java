/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.wheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class ToggleWheelPosition extends InstantCommand {
    public ToggleWheelPosition() {
    }

    @Override
    public void initialize() {
        if(Robot.getRobotContainer().getDriveTrain().getDragonFlyWheelState() == DriveTrain.WheelState.Lowered)
            Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Lowered);
        else{
            Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Raised);
            Robot.getRobotContainer().getDriveTrain().setDefaultDrive();
        }
    }
}
