/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class SpeedReductionMoveToAngle extends CommandBase {
    /**
     * Moves certain amount of meters to a certain local angle
     * using speed reduction control with DragonFly move.
     */
    double meters, angle, maxPower, totalDistance;
    public SpeedReductionMoveToAngle(double meters, double angle, double maxPower) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        this.meters = meters;
        this.angle = angle;
        this.maxPower = maxPower;
        totalDistance = 0;
    }

    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Lowered);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
