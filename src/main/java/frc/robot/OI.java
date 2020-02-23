/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.robotActions.DisengageLosenRopeAndActivatePulleyMotors;
import frc.robot.commands.subsystemCommands.chassis.QuickTurn;
import frc.robot.commands.subsystemCommands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.subsystemCommands.chassis.drivingModes.ChassisSetPivoting;
import frc.robot.commands.subsystemCommands.chassis.drivingModes.ChassisToggleTransmissionMode;
import frc.robot.commands.subsystemCommands.chassis.wheel.ToggleWheelPosition;
import frc.robot.commands.subsystemCommands.climber.ActivateManualWinch;
import frc.robot.commands.subsystemCommands.controlPanel.PositionServo;
import frc.robot.commands.subsystemCommands.controlPanel.PositionServoToZero;
import frc.robot.commands.subsystemCommands.shooter.ShootFromInitiationLine;
import frc.robot.resources.TecbotController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static OI instance;
    TecbotController pilot, copilot;

    public OI() {
        pilot = new TecbotController(0);
        copilot = new TecbotController(1);

    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    public void configureButtonBindings() {
        //PILOT STARTS
        //put here all

        pilot.whileHeld(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getRearIntakeAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getIntakesAndTransportOff());

        pilot.whileHeld(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getFrontIntakeAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getIntakesAndTransportOff());

        pilot.whenPressed(TecbotController.ButtonType.A, new ToggleWheelPosition());

        pilot.whenPressed(TecbotController.ButtonType.B, new ChassisSetPivoting());
        pilot.whenReleased(TecbotController.ButtonType.B, new ChassisSetDefaultDrive());

        pilot.whenPressed(TecbotController.ButtonType.X, RobotActionsCatalog.getInstance().getAllSystemsOff());

        pilot.whenPressed(TecbotController.ButtonType.Y, new QuickTurn());

        pilot.whenPressed(TecbotController.ButtonType.BACK, new PositionServoToZero());

        pilot.whenPressed(TecbotController.ButtonType.LS, new PositionServo());

        pilot.whenPressed(TecbotController.ButtonType.START, new ChassisToggleTransmissionMode());

        //POV a.k.a. D-PAD
        //POV ^
        //    |
        pilot.whenPressed(TecbotController.ButtonType.POV_0, RobotActionsCatalog.getInstance().getShootFromInitiationLineCompensate());

        //POV -->
        pilot.whenPressed(TecbotController.ButtonType.POV_90, RobotActionsCatalog.getInstance().getShootFromTrenchCompensate());

        //POV <--
        pilot.whenPressed(TecbotController.ButtonType.POV_270, RobotActionsCatalog.getInstance().getShootFromTargetZoneCompensate());
        //pilot.whenPressed(TecbotController.ButtonType.POV_270, new ShootFromInitiationLine());

        //POV |
        //    ¿
        pilot.whileHeld(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getFrontOutTakeAndTransport());
        pilot.whenReleased(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getAllSystemsOff());

        // pilot.whenPressed(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getTransportDeflectorOff());

        // pilot.whenPressed(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getNoPIDShootTrenchAndTransport());

        //PILOT ENDS

        //COPILOT STARTS

        copilot.whileHeld(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getRearIntakeAndShootBottomPort());
        copilot.whenReleased(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getAllSystemsOff());

        copilot.whileHeld(TecbotController.ButtonType.B, RobotActionsCatalog.getInstance().getIntakeFromFeederAndTransport());
        copilot.whenReleased(TecbotController.ButtonType.B, RobotActionsCatalog.getInstance().getAllSystemsOff());

        copilot.whenPressed(TecbotController.ButtonType.X, new ActivateManualWinch());
        copilot.whenPressed(TecbotController.ButtonType.Y, new DisengageLosenRopeAndActivatePulleyMotors());


        //COPILOT ENDS
    }

    public TecbotController getPilot() {
        return pilot;
    }

    public TecbotController getCopilot() {
        return copilot;
    }
}