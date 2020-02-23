/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootFromInitiationLine extends CommandBase {

    boolean isFinished = false;

    public ShootFromInitiationLine() {
        addRequirements(Robot.getRobotContainer().getShooter());
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (RobotMap.SHOOTER_PID_SHOOTER_IS_AVAILABLE) {
            ShooterPosition position = ShooterPosition.INITIATION_LINE;
            Robot.getRobotContainer().getShooter().setAnglerDegrees(position);
            Robot.getRobotContainer().getShooter().setShootingSpeed(position);
            Robot.getRobotContainer().getShooter().enable();

        } else {
            Robot.getRobotContainer().getShooter().disable();
        }
    }

    @Override
    public void execute() {
        if (RobotMap.SHOOTER_PID_SHOOTER_IS_AVAILABLE) {
            isFinished = true;
            return;
        }
        double currentSpeed = Robot.getRobotContainer().getTecbotSensors().getEncoder(TecbotSensors.SubsystemType.SHOOTER).getRate(),
                error = currentSpeed - TecbotConstants.SHOOTER_INITIATION_LINE_SHOOTING_SPEED;

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
