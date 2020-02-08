/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.robotActions.intakeTransport.RearIntakeAndShootBottomPort;
import frc.robot.resources.TecbotController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static OI instance;
    TecbotController pilot, copilot;

    public OI() {

    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    public void configureButtonBindings() {
        //PILOT STARTS
        //put here all
        pilot = new TecbotController(0);

        pilot.whileHeld(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getRearIntakeAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getAllSystemsOff());

        pilot.whileHeld(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getFrontIntakeAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getAllSystemsOff());

        pilot.whenPressed(TecbotController.ButtonType.X, RobotActionsCatalog.getInstance().getAllSystemsOff());

        //POV a.k.a. D-PAD
        pilot.whileHeld(TecbotController.ButtonType.POV_0, RobotActionsCatalog.getInstance().getShootTargetZoneAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.POV_0, RobotActionsCatalog.getInstance().getAllSystemsOff());

        pilot.whileHeld(TecbotController.ButtonType.POV_90, RobotActionsCatalog.getInstance().getShootInitiationLineAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.POV_90, RobotActionsCatalog.getInstance().getAllSystemsOff());

        pilot.whileHeld(TecbotController.ButtonType.POV_270, RobotActionsCatalog.getInstance().getShootTrenchAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.POV_270, RobotActionsCatalog.getInstance().getAllSystemsOff());

        //PILOT ENDS

        //COPILOT STARTS
        copilot = new TecbotController(1);

        copilot.whileHeld(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getAllSystemsOff());
        copilot.whenReleased(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getAllSystemsOff());

        copilot.whileHeld(TecbotController.ButtonType.B,RobotActionsCatalog.getInstance().getIntakeFromFeederAndTransport());
        copilot.whenReleased(TecbotController.ButtonType.B, RobotActionsCatalog.getInstance().getAllSystemsOff());

        //TODO create whenPressed for double press for 'x' in copilot initiateClimbingMode()
        /*TODO initiateCLimbingMode() should create a command which does not end and maps
        LT and RT values to left and right pulley system.*/


        //COPILOT ENDS
    }

    public TecbotController getPilot() {
        return pilot;
    }

    public TecbotController getCopilot() {
        return copilot;
    }
}