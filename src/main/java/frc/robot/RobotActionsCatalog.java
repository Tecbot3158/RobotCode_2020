package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.robotActions.RA_ALL_SYSTEMS_OFF;
import frc.robot.commands.robotActions.RA_FRONT_INTAKE_AND_TRANSPORT_ONLY;
import frc.robot.commands.robotActions.RA_REAR_INTAKE_AND_TRANSPORT_ONLY;

public class RobotActionsCatalog {
    CommandGroupBase ALL_SYSTEMS_OFF = new RA_ALL_SYSTEMS_OFF();
    CommandGroupBase FRONT_INTAKE_AND_TRANSPORT_ONLY = new RA_FRONT_INTAKE_AND_TRANSPORT_ONLY();
    CommandGroupBase REAR_INTAKE_AND_TRANSPORT_ONLY = new RA_REAR_INTAKE_AND_TRANSPORT_ONLY();


}
