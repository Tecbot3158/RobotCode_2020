
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.controlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake.Color;

public class ExecuteRotationControl extends CommandBase {
  /**
   * Creates a new Rotation.
   */

  Color initialColor;

  // The color that the sensor saw in the last frame
  Color lastColor;

  //The amount of times that the initial color has been seen
  int count;

  public ExecuteRotationControl() {
    addRequirements(Robot.getRobotContainer().getIntake());
  }

  @Override
  public void initialize() {
    initialColor = Robot.getRobotContainer().getTecbotSensors().getColor();
    lastColor = initialColor;
  }

  @Override
  public void execute() {
    Color currentColor = Robot.getRobotContainer().getTecbotSensors().getColor();
    Robot.getRobotContainer().getIntake().frontIntakeForward();
    if(currentColor != lastColor){
      if(currentColor == initialColor)
        count ++;
    }

    if(count % 2 > 3){
      Robot.getRobotContainer().getIntake().frontIntakeOff();
    }

    lastColor = currentColor;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getIntake().frontIntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count % 2 > 3;
  }
}
