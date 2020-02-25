/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.subsystemCommands.intakes.DefaultCommandIntakes;
import frc.robot.commands.subsystemCommands.intakes.frontIntakes.FrontIntakeSolenoidOn;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeSolenoidOff;
import frc.robot.commands.subsystemCommands.intakes.rearIntakes.RearIntakeSolenoidOn;
import frc.robot.commands.subsystemCommands.pctower.DefaultCommandTransportationSystem;
import frc.robot.commands.subsystemCommands.powerCellCounter.DefaultCommandPowerCellCounter;
import frc.robot.commands.subsystemCommands.chassis.DefaultDrive;
import frc.robot.commands.subsystemTester.*;
import frc.robot.subsystems.chassis.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

    private static RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    public static int currentMotorBeingTested = 0;
    public static int currentSecondMotorBeingTested = 0;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        SmartDashboard.putBoolean("CLIMB", false);
        SmartDashboard.putNumber("servo", 0);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        //coast.
        if (RobotMap.DRIVE_TRAIN_DRAGON_FLY_IS_AVAILABLE)
            getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(CANSparkMax.IdleMode.kCoast);

        getRobotContainer().configureButtonBindings();
        getRobotContainer().getTecbotSensors().initializeAllSensors();

        Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(CANSparkMax.IdleMode.kBrake, RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_PORT);
        Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(CANSparkMax.IdleMode.kBrake, RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_PORTS);
        Robot.getRobotContainer().getDriveTrain().setCANSparkMaxMotorsState(CANSparkMax.IdleMode.kBrake, RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_PORTS);

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);

        //camera.setExposureManual(79);


    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        getRobotContainer().getTecbotSensors().sensorsPeriodic();
        SmartDashboard.putNumber("gyro", getRobotContainer().getTecbotSensors().getYaw());
        SmartDashboard.putBoolean("isSpeed", getRobotContainer().getDriveTrain().getTransmissionMode() == DriveTrain.TransmissionMode.speed);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().run();
        m_autonomousCommand = getRobotContainer().getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("XCOUNT", getRobotContainer().getClimber().getxWhenPressedCount());
        SmartDashboard.putBoolean("PULLEY", false);


        //getRobotContainer().getDriveTrain().setOrientation(true);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }


        Robot.getRobotContainer().getIntake().frontIntakeSolenoidLowered();
        Robot.getRobotContainer().getIntake().rearIntakeOff();


        getRobotContainer().getDriveTrain().setDefaultCommand(new DefaultDrive());
        getRobotContainer().getIntake().setDefaultCommand(new DefaultCommandIntakes());
        getRobotContainer().getTransportationSystem().setDefaultCommand(new DefaultCommandTransportationSystem());
        getRobotContainer().getPowerCellCounter().setDefaultCommand(new DefaultCommandPowerCellCounter());
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        OI.getInstance().getPilot().run();
        SmartDashboard.putNumber("gyro", getRobotContainer().getTecbotSensors().getYaw());


    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();

        System.out.println(OI.getInstance().getPilot().getTriggers());
        SmartDashboard.putData(new TestClimber());
        SmartDashboard.putData(new TestIntake());
        SmartDashboard.putData(new TestPCT());
        SmartDashboard.putData(new TestShooter());
        SmartDashboard.putData(new ExitTestingMode());
        SmartDashboard.putData(new TestFrontIntake());
        SmartDashboard.putData(new TestRearIntake());
        SmartDashboard.putData(new TestRightShooter());
        SmartDashboard.putData(new TestLeftShooter());
        SmartDashboard.putData(new TestPCTSpecific());
        SmartDashboard.putData(new ChangeCurrentMotor());
        SmartDashboard.putData(new ChangeSecondCurrentMotor());
        SmartDashboard.putData(new TestLeftClimber());
        SmartDashboard.putData(new TestRightClimber());

        //SmartDashboard.putData(RobotActionsCatalog.getInstance().getFrontOutTakeAndTransport());
        SmartDashboard.putData(new RearIntakeSolenoidOn());
        SmartDashboard.putData(new RearIntakeSolenoidOff());


    }

    public static RobotContainer getRobotContainer() {
        return m_robotContainer;
    }
}
