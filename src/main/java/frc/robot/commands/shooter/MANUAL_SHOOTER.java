/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MANUAL_SHOOTER extends PIDCommand {
  /**
   * Creates a new MANUAL_SHOOTER.
   */
  public MANUAL_SHOOTER() {
    super(
        // The controller that the command will use
        new PIDController(TecbotConstants.K_SHOOTER_P, TecbotConstants.K_SHOOTER_I, TecbotConstants.K_SHOOTER_D),
        // This should return the measurement
        () -> RobotContainer.getShooter().getMeasurement(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          RobotContainer.getShooter().useOutput(output);

        });
       RobotContainer.getShooter().setManualAngler
        (RobotContainer.getOI().getPilot().getRawAxis(4), 
        RobotContainer.getOI().getPilot().getRawAxis(5));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
