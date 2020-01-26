/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemStateHandler extends SubsystemBase {



    /**
     * The SubsystemStateHandler manages all the Tecbot
     * Robot Subsystems, as well as controlling which possible
     * Robot Command State is running.
     */
    public SubsystemStateHandler() {
        SequentialCommandGroup command = RobotCurrentStateCommand.FRONT_INTAKE_AND_TRANSPORT_ONLY.command;
        CommandScheduler.getInstance().schedule(true,command);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
