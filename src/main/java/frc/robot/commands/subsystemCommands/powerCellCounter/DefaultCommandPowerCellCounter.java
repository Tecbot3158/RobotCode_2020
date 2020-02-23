/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystemCommands.powerCellCounter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotController;

public class DefaultCommandPowerCellCounter extends CommandBase {
    private boolean currentStateFrontIntakeIRSensor = false, currentStateRearIntakeIRSensor = false,
            currentStateShooterIRSensor = false, previousStateFrontIntakeIRSensor = false,
            previousStateRearIntakeIRSensor = false, previousStateShooterIRSensor = false,
            red = false, green = false, blue = false;


    int frameCounter = 1, currentCount = 0;

    /**
     * Creates a new Command.
     */
    public DefaultCommandPowerCellCounter() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.getRobotContainer().getPowerCellCounter());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        frameCounter++;

        currentCount = Robot.getRobotContainer().getPowerCellCounter().getPowerCellCount();
        currentStateFrontIntakeIRSensor = Robot.getRobotContainer().getTecbotSensors().getIRSensorStateFrontIntake();
        currentStateRearIntakeIRSensor = Robot.getRobotContainer().getTecbotSensors().getIRSensorStateRearIntake();
        currentStateShooterIRSensor = Robot.getRobotContainer().getTecbotSensors().getIRSensorStateShooter();


        //change detected in intake (detected power cell and now is not present)
        //if condition true, a power cell has gotten inside the transportation system.
        if (!previousStateFrontIntakeIRSensor && currentStateFrontIntakeIRSensor)
            Robot.getRobotContainer().getPowerCellCounter().addToPowerCellCount();

        if (!previousStateRearIntakeIRSensor && currentStateRearIntakeIRSensor)
            Robot.getRobotContainer().getPowerCellCounter().addToPowerCellCount();

        //change detected in shooter (detected power cell and now is not present)
        //if condition true, a power cell has left the shooter
        if (!previousStateShooterIRSensor && currentStateShooterIRSensor)
            Robot.getRobotContainer().getPowerCellCounter().subtractFromPowerCellCount();

        switch (currentCount) {
            case 1:
                //green solid
                red = false;
                green = true;
                blue = false;
                break;
            case 2:
                //green flickering
                if (Math.module(frameCounter, RobotMap.POWER_CELL_COUNTER_GREEN_FLICKERING_FRAME_COUNT) == 0) {
                    red = false;
                    green = true;
                    blue = false;
                    frameCounter = 0;
                } else {
                    red = false;
                    green = false;
                    blue = false;
                }
                break;
            case 3:
                //yellow is red + green
                red = true;
                green = true;
                blue = false;
                break;
            case 4:
                //red flickering
                if (Math.module(frameCounter, RobotMap.POWER_CELL_COUNTER_RED_FLICKERING_FRAME_COUNT) == 0) {
                    red = true;
                    green = false;
                    blue = false;
                    frameCounter = 0;
                } else {
                    red = false;
                    green = false;
                    blue = false;
                }
                break;
            case 5:
                //red solid
                red = true;
                green = false;
                blue = false;
                break;
            //acts as default case and '0'
            default:
                //off solid
                red = false;
                green = false;
                blue = false;
                break;

        }

        Robot.getRobotContainer().getPowerCellCounter().setColor(red, green, blue);

        previousStateFrontIntakeIRSensor = currentStateFrontIntakeIRSensor;
        previousStateRearIntakeIRSensor = currentStateRearIntakeIRSensor;
        previousStateShooterIRSensor = currentStateShooterIRSensor;

        SmartDashboard.putNumber("PC count", currentCount);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
