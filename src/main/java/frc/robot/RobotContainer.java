/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.resources.TecbotSensors;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.pctower.TransportationSystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public static DriveTrain driveTrain;
    public static OI oi;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public TransportationSystem transportationSystem;


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        transportationSystem = new TransportationSystem();
        
        // Configure the button bindings
        configureButtonBindings();
        TecbotSensors.initializeAllSensors();
        driveTrain = new DriveTrain();
        oi = new OI();


    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        oi.configureButtonBindings();
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_chooser.getSelected();
    }

    public static DriveTrain getDriveTrain(){
        return driveTrain;
    }

    public static OI getOI(){
        return oi;
    }

    public TransportationSystem getTransportationSystem(){
        return transportationSystem;
    }

}
