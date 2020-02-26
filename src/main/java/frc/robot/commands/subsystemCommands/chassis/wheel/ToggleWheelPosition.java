/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.chassis.wheel;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.chassis.DriveTrain;

public class ToggleWheelPosition extends InstantCommand {
    public ToggleWheelPosition() {
    }

    @Override
    public void initialize() {
        if (Robot.getRobotContainer().getDriveTrain().getDragonFlyWheelState() == DriveTrain.WheelState.Lowered) {
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(true, RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_PORT);
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(true, RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_PORTS);
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(true, RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_PORTS);
            Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Raised);
        } else {
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(false, RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_PORT);
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(false, RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_PORTS);
            Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(false, RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_PORTS);

            Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Lowered);
            Robot.getRobotContainer().getDriveTrain().setDefaultDrive();
        }
    }
}
