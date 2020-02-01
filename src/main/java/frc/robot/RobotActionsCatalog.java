package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.robotActions.AllSystemsOff;
import frc.robot.commands.robotActions.FrontIntakeAndTransportOnly;
import frc.robot.commands.robotActions.RearIntakeAndTransportOnly;

public class RobotActionsCatalog {
    private static RobotActionsCatalog instance;


    CommandGroupBase allSystemsOff;
    CommandGroupBase frontIntakeAndTransportOnly;
    CommandGroupBase rearIntakeAndTransportOnly;

    public RobotActionsCatalog() {
        allSystemsOff = new AllSystemsOff();
        frontIntakeAndTransportOnly = new FrontIntakeAndTransportOnly();
        rearIntakeAndTransportOnly = new RearIntakeAndTransportOnly();

    }

    public CommandGroupBase getFrontIntakeAndTransportOnly() {
        return frontIntakeAndTransportOnly;
    }

    public CommandGroupBase getRearIntakeAndTransportOnly() {
        return rearIntakeAndTransportOnly;
    }

    public CommandGroupBase getAllSystemsOff() {
        return allSystemsOff;
    }

    public static RobotActionsCatalog getInstance() {
        if (instance == null) instance = new RobotActionsCatalog();
        return instance;
    }

}