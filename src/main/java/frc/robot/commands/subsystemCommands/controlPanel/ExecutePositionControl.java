/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.controlPanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.Math;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Color;

public class ExecutePositionControl extends CommandBase {
  /**
   * Executes position control
   */

  // The id of the target color
  int targetColorID;

  // The difference in colors between the actual color and the target color
  int deltaColors;
  public ExecutePositionControl() {
    addRequirements(Robot.getRobotContainer().getIntake());
  }

  @Override
  public void initialize() {
    int currentColorID;

    String fmsMessage = DriverStation.getInstance().getGameSpecificMessage();
    Color fmsTargetColor;
    switch (fmsMessage){
      case "R":
        fmsTargetColor = Color.RED;
        break;
      case "G":
        fmsTargetColor = Color.GREEN;
        break;
      case "B":
        fmsTargetColor = Color.BLUE;
        break;
      case "Y":
        fmsTargetColor = Color.YELLOW;
        break;
      default:
        DriverStation.reportWarning("Color provided by FMS not recognized, position control " +
                "won't be done autonomously", true);
        fmsTargetColor = null;
    }

    if(fmsTargetColor == null){
      targetColorID = -1;
    }else {
      // Since what our sensor is seeing is 90 degrees away (2 colors) from what the fms sensor is seeing
      // we need to make a conversion in order to know what color needs to be in our sensor so that the fms
      // detects position control completed.
      targetColorID = Intake.getIDFromColor(fmsTargetColor) - 2;
    }
      currentColorID = Intake.getIDFromColor(Robot.getRobotContainer().getTecbotSensors().getColor());

    // The difference in colors between the actual color and the target color if rotating clockwise
    int clockwiseDifference = Math.module(targetColorID - currentColorID, 4);

    // The difference in colors between the actual color and the target color if rotating counterclockwise
    int counterclockwiseDifference = Math.module(currentColorID - targetColorID, 4);
    if(clockwiseDifference < counterclockwiseDifference)
      deltaColors = clockwiseDifference;
    else
      deltaColors = -counterclockwiseDifference;
  }

  @Override
  public void execute() {
    if(deltaColors > 0){
      Robot.getRobotContainer().getIntake().frontIntakeForward();
    }else {
      Robot.getRobotContainer().getIntake().frontIntakeReverse();
    }
    if(Intake.getIDFromColor(Robot.getRobotContainer().getTecbotSensors().getColor()) == targetColorID){
      Robot.getRobotContainer().getIntake().frontIntakeOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getIntake().frontIntakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Intake.getIDFromColor(Robot.getRobotContainer().getTecbotSensors().getColor()) == targetColorID;
  }
}
