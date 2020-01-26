package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

public enum RobotCurrentStateCommand {
    FRONT_INTAKE_AND_TRANSPORT_ONLY(RobotMap.FRONT_INTAKE_AND_TRANSPORT_ONLY),
    REAR_INTAKE_AND_TRANSPORT_ONLY(RobotMap.FRONT_INTAKE_AND_TRANSPORT_ONLY);


    public SequentialCommandGroup command;

    RobotCurrentStateCommand(SequentialCommandGroup commandGroup){
        this.command = commandGroup;
    }
}
