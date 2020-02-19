/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous.speedReduction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;
import frc.robot.subsystems.chassis.DriveTrain;
import org.opencv.core.Mat;

public class SpeedReductionMoveSwerve extends CommandBase {
    /**
     * Moves certain amount of meters to a certain absolute angle
     * using speed reduction control with DragonFly move.
     */
    double targetMeters, angle, maxPower, totalDistance, targetAngle;
    double lastChassisCount, lastMiddleCount;

    boolean onTarget;

    public SpeedReductionMoveSwerve(double meters, double angle, double maxPower, double targetAngle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        this.targetMeters = Math.abs(meters);
        this.angle = angle;
        this.maxPower = maxPower;
        this.targetAngle = targetAngle;
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

        double currentChassisCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS);
        double currentMiddleCount = Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(TecbotSensors.SubsystemType.MIDDLE_CHASSIS);

        double deltaDistanceChassis = TecbotConstants.K_CHASSIS_ENCODER_TO_METERS * (lastChassisCount - currentChassisCount);
        double deltaMiddleChassis = TecbotConstants.K_MIDDLE_WHEEL_ENCODER_TO_METERS * (lastMiddleCount - currentMiddleCount);

        totalDistance += Math.hypot(deltaDistanceChassis, deltaMiddleChassis);

        double power = Math.clamp((targetMeters - totalDistance) / TecbotConstants.CHASSIS_SWERVE_MAX_DISTANCE,
                 -1, 1);
        double turn = Math.clamp( (targetAngle - Robot.getRobotContainer().getTecbotSensors().getYaw()) / TecbotConstants.CHASSIS_TURN_MAX_DISTANCE,
                -1 , 1);

        double x = Math.sin(angle) * power;
        double y = Math.cos(angle) * power;

        Robot.getRobotContainer().getDriveTrain().swerveMove(x,y,turn);

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
