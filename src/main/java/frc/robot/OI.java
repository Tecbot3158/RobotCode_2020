/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ResetGyro;
import frc.robot.commands.chassis.QuickTurn;
import frc.robot.commands.chassis.drivingModes.ToggleMecanum;
import frc.robot.commands.chassis.drivingModes.TogglePivoting;
import frc.robot.commands.chassis.drivingModes.ToggleSwerve;
import frc.robot.commands.chassis.wheel.LowerWheel;
import frc.robot.commands.chassis.wheel.RiseWheel;
import frc.robot.resources.TecbotController.ButtonType;
import frc.robot.resources.TecbotController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private TecbotController pilot;
    private static OI instance;

    public OI() {
        pilot = new TecbotController(0);
    }

    public OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    public void configureButtonBindings() {
        pilot.whenPressed(ButtonType.A, new LowerWheel());
        pilot.whenPressed(ButtonType.B, new RiseWheel());
        pilot.whenPressed(ButtonType.Y, new QuickTurn());
        pilot.whenPressed(ButtonType.RS, new ToggleSwerve());
        pilot.whenPressed(ButtonType.LS, new ToggleMecanum());
        pilot.whenPressed(ButtonType.RB, new TogglePivoting());
        pilot.whenReleased(ButtonType.RB, new TogglePivoting());
        pilot.whenReleased(ButtonType.LB, new ResetGyro());

    }

    public TecbotController getPilot() {
        return pilot;
    }
}
