/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class ShootingFromInitiationLine extends InstantCommand {
  double output, setpoint; 
  public ShootingFromInitiationLine() {
    addRequirements(Robot.getRobotContainer().getShooter());
    
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    ShooterPosition position = ShooterPosition.INITIATION_LINE;
    Robot.getRobotContainer().getShooter().setShootingSpeed(position);
    Robot.getRobotContainer().getShooter().setAnglerDegrees(position);
    Robot.getRobotContainer().getShooter().enable();
    Robot.getRobotContainer().getShooter().useOutput(output, setpoint);
    Robot.getRobotContainer().getShooter().getMeasurement();
    

  }
}
