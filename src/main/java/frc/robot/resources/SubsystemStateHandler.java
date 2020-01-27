/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.*;

public class SubsystemStateHandler extends SubsystemBase {

    RobotCurrentStateCommand currentCommand;

    /**
     * The SubsystemStateHandler manages all the Tecbot Robot Subsystems, as well as
     * controlling which possible Robot Command State is running.
     */
    public SubsystemStateHandler() {
        currentCommand = RobotCurrentStateCommand.ALL_SYSTEMS_OFF;
    }

    public void setCurrentCommand(RobotCurrentStateCommand stateCommand) {
        if (!currentCommand.getCommand().isFinished()) {
            CommandScheduler.getInstance().cancel(currentCommand.getCommand());

            currentCommand.addToCancelVariable();
            currentCommand.sendCommandData(true, true);

        }
        // this line was written because of this:
        // https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html
        CommandGroupBase.clearGroupedCommand(currentCommand.getCommand());
        currentCommand = stateCommand;
        CommandScheduler.getInstance().schedule(true, currentCommand.getCommand());
        currentCommand.addToScheduleVariable();
        currentCommand.sendCommandData(true, true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}