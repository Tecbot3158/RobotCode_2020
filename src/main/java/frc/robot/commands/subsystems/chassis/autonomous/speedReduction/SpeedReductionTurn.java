/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpeedReductionTurn extends CommandBase {
    /**
     * Creates a new Command.
     */
    double target, maxPower;

    public SpeedReductionTurn(double targetAngle, double maxPower) {
        // Use addRequirements() here to declare subsystem dependencies.
        target = targetAngle;
        this.maxPower = maxPower;
        addRequirements(Robot.getRobotContainer().getDriveTrain());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Robot.getRobotContainer().getDriveTrain().turn(target, maxPower);
    }
}
