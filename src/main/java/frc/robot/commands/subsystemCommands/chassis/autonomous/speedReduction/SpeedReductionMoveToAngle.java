/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;
import frc.robot.subsystems.chassis.DriveTrain;

public class SpeedReductionMoveToAngle extends CommandBase {
    /**
     * Moves certain amount of meters to a certain local angle
     * using speed reduction control with DragonFly move.
     */
    double targetMeters, angle, maxPower, totalDistance;
    double lastChassisCount, lastMiddleCount;

    boolean onTarget;
    public SpeedReductionMoveToAngle(double meters, double angle, double maxPower) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        this.targetMeters = meters;
        this.angle = angle;
        this.maxPower = maxPower;
        totalDistance = 0;
        onTarget = false;
    }

    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setDragonFlyWheelState(DriveTrain.WheelState.Lowered);
        lastChassisCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS);
        lastMiddleCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.MIDDLE_CHASSIS);
    }

    @Override
    public void execute() {
        Robot.getRobotContainer().getDriveTrain().driveToAngle(angle, maxPower, 0);

        double currentChassisCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS);
        double currentMiddleCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.MIDDLE_CHASSIS);

        double deltaDistanceChassis = TecbotConstants.K_CHASSIS_ENCODER_TO_METERS * (lastChassisCount - currentChassisCount);
        double deltaMiddleChassis = TecbotConstants.K_MIDDLE_WHEEL_ENCODER_TO_METERS * (lastMiddleCount - currentMiddleCount);

        double power = Math.clamp((targetMeters - totalDistance) / TecbotConstants.CHASSIS_SWERVE_MAX_DISTANCE
                , -1, 1);

        Robot.getRobotContainer().getDriveTrain().driveToAngle(angle, power, 0);

        totalDistance += Math.hypot(deltaDistanceChassis, deltaMiddleChassis);

        if (Math.abs(targetMeters - totalDistance) <= TecbotConstants.CHASSIS_SWERVE_ARRIVE_OFFSET)
            onTarget = true;

    }

    @Override
    public void end(boolean interrupted) {
        Robot.getRobotContainer().getDriveTrain().stop();
    }

    @Override
    public boolean isFinished() {
        return onTarget;
    }
}
