/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootManual extends InstantCommand {
    double manualSpeed;
    double manualAngle;

    public ShootManual(double manualSpeed, double manualAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getShooter());

        this.manualSpeed = manualSpeed;
        this.manualAngle = manualAngle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getShooter().disable();
        Robot.getRobotContainer().getShooter().setManualShooter(manualSpeed);
        Robot.getRobotContainer().getShooter().setManualShooter(manualAngle);
    }

}
