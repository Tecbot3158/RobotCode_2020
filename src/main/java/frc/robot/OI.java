/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.resources.TecbotController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static OI instance;
    TecbotController pilot;

    public OI() {

    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    public void configureButtonBindings() {
        //put here all
        pilot = new TecbotController(0);

        pilot.whenPressed(TecbotController.ButtonType.X, RobotActionsCatalog.getInstance().getAllSystemsOff());
        //pilot.whenPressed(TecbotController.ButtonType.POV_0, RobotActionsCatalog.getInstance().allSystemsOff);
        //pilot.whenReleased(TecbotController.ButtonType.POV_0, new TestWhenReleasedCommand());
        //pilot.whileHeld(TecbotController.ButtonType.POV_0, new TestWhileHeld());
    }

    public TecbotController getPilot() {
        return pilot;
    }
}