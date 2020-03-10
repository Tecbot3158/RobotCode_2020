/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.robotActions.DisengageLosenRopeAndActivatePulleyMotors;
import frc.robot.commands.subsystemCommands.chassis.QuickTurn;
import frc.robot.commands.subsystemCommands.chassis.drivingModes.*;
import frc.robot.commands.subsystemCommands.chassis.wheel.ToggleWheelPosition;
import frc.robot.commands.subsystemCommands.controlPanel.PositionServoToZero;
import frc.robot.resources.TecbotController;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private static OI instance;
    TecbotController pilot, copilot;

    public OI() {
        pilot = new TecbotController(0, TecbotController.TypeOfController.XBOX);
        copilot = new TecbotController(1, TecbotController.TypeOfController.PS4);

    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }

    public void configureButtonBindings() {
        Intake intakes = Robot.getRobotContainer().getIntake();
        //PILOT STARTS
        //put here all

        //pilot.whileHeld(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getRearIntakeAndTransport());
        //pilot.whenReleased(TecbotController.ButtonType.LB, RobotActionsCatalog.getInstance().getIntakesAndTransportOff());

        //pilot.whileHeld(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getFrontIntakeAndTransport());
        //pilot.whenReleased(TecbotController.ButtonType.RB, RobotActionsCatalog.getInstance().getIntakesAndTransportOff());
        pilot.whenPressed(TecbotController.ButtonType.RB, new InstantCommand(intakes::frontIntakeToggleSolenoid));

        pilot.whenPressed(TecbotController.ButtonType.LB, new InstantCommand(intakes::rearIntakeToggleSolenoid));


        //pilot.whenPressed(TecbotController.ButtonType.A, new ToggleWheelPosition());

        pilot.whenPressed(TecbotController.ButtonType.B, new ChassisSetPivoting());
        pilot.whenReleased(TecbotController.ButtonType.B, new ChassisSetDefaultDrive());

        pilot.whenPressed(TecbotController.ButtonType.X, RobotActionsCatalog.getInstance().getAllSystemsOff());

        //pilot.whenPressed(TecbotController.ButtonType.Y, new QuickTurn());

        //pilot.whenPressed(TecbotController.ButtonType.BACK, new PositionServoToZero());

        //pilot.whenPressed(TecbotController.ButtonType.LS, new PositionServo());
        //pilot.whenPressed(TecbotController.ButtonType.LS, new ToggleMecanum());

        //pilot.whenPressed(TecbotController.ButtonType.RS, new ToggleSwerve());

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
        //pilot.whileHeld(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getFrontOutTakeAndTransport());
        pilot.whenPressed(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getTransportationSystemShootingSpeed());

        //pilot.whenReleased(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getAllSystemsOff());

        // pilot.whenPressed(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getTransportDeflectorOff());

        // pilot.whenPressed(TecbotController.ButtonType.POV_180, RobotActionsCatalog.getInstance().getNoPIDShootTrenchAndTransport());

        //PILOT ENDS

        /*COPILOT STARTS

         */
        //CLIMBING STARTS
        copilot.whenPressed(TecbotController.ButtonType.START, RobotActionsCatalog.getInstance().getAddToXAndActivateManualWinch());

        //copilot.whenPressed(TecbotController.ButtonType.BACK, new DisengageLosenRopeAndActivatePulleyMotors());
        /*CLIMBING ENDS.
         */

        copilot.whenPressed(TecbotController.ButtonType.LB, new ResetGyro());


        //COPILOT ENDS
    }

    public TecbotController getPilot() {
        return pilot;
    }

    public TecbotController getCopilot() {
        return copilot;
    }
}