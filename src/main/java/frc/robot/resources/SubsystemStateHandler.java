/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class SubsystemStateHandler extends SubsystemBase {


    SequentialCommandGroup currentCommand;

    /**
     * The SubsystemStateHandler manages all the Tecbot
     * Robot Subsystems, as well as controlling which possible
     * Robot Command State is running.
     */
    public SubsystemStateHandler() {
        currentCommand = RobotCurrentStateCommand.ALL_SYSTEMS_OFF.getCommand();
    }

    public void setCurrentCommand(RobotCurrentStateCommand stateCommand) {
        if (!currentCommand.isFinished()) CommandScheduler.getInstance().cancel(currentCommand);
        currentCommand = stateCommand.getCommand();
        CommandScheduler.getInstance().schedule(true, currentCommand);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
