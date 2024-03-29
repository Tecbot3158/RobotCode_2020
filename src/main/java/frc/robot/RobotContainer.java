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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.subsystemCommands.chassis.DefaultDrive;
import frc.robot.commands.subsystemCommands.chassis.autonomous.speedReduction.SpeedReductionStraight;
import frc.robot.resources.TecbotSensors;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SharedMotors;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pctower.TransportationSystem;
import frc.robot.subsystems.powerCellCounter.PowerCellCounter;
import frc.robot.subsystems.shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private SharedMotors sharedMotors;
    private DriveTrain driveTrain;
    private Climber climber;
    private Intake intake;
    private Shooter shooter;
    private TransportationSystem transportationSystem;
    private PowerCellCounter powerCellCounter;
    private TecbotSensors tecbotSensors;

    SendableChooser<Command> m_chooser = new SendableChooser<>();


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        driveTrain = new DriveTrain();
        climber = new Climber();
        intake = new Intake();
        shooter = new Shooter();
        transportationSystem = new TransportationSystem();
        sharedMotors = new SharedMotors();
        // Remember that shared motors are the motors that belong to the climber and the shooter.

        tecbotSensors = new TecbotSensors();

        powerCellCounter = new PowerCellCounter();

        // configureButtonBindings() called after instantiating all subsystems and
        // RobotContainer constructor.

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {
        CommandScheduler.getInstance().setDefaultCommand(driveTrain, new DefaultDrive());
        OI.getInstance().configureButtonBindings();
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

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public TecbotSensors getTecbotSensors() {
        return tecbotSensors;
    }

    public SharedMotors getSharedMotors() {
        return sharedMotors;
    }

    public Climber getClimber() {
        return climber;
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public TransportationSystem getTransportationSystem() {
        return transportationSystem;
    }

    public PowerCellCounter getPowerCellCounter() {
        return powerCellCounter;
    }
}
