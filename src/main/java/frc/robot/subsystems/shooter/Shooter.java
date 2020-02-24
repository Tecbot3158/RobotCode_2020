/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */

package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors.SubsystemType;


public class Shooter extends PIDSubsystem {

    Servo anglerServo;


    SimpleMotorFeedforward feed;

    double speed;
    double angle;

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        super(
                // The PIDController used by the subsystem
                new PIDController(TecbotConstants.K_SHOOTER_P,
                        TecbotConstants.K_SHOOTER_I,
                        TecbotConstants.K_SHOOTER_D));

        feed = new SimpleMotorFeedforward(TecbotConstants.KS_VOLTS, TecbotConstants.KV_VOLT_SECONDS_PER_ROTATION);
        getController().setTolerance(0.5);
        // distance per pulse on encoder is 1/30,000 set directly on get trate

        //anglerServo = RobotConfigurator.buildServo(RobotMap.SHOOTER_ANGLER_PORT);


    }

    @Override
    public void useOutput(double output, double setpoint) {
        // Use the output here
        //anglerServo.setAngle(angle);

        //double power =  feed.calculate(setpoint);
        //System.out.println("OUTPUT --> " + output + " // " + setpoint + " //  " + power);


        // do not take PID into account, voltage is enough
        shoot(speed);
    }

    @Override
    public double getMeasurement() {
        // Return the process variable measurement here
        //return Robot.getRobotContainer().getTecbotSensors().getEncoderRaw(SubsystemType.SHOOTER);
        //System.out.println("-> encoder " +  Robot.getRobotContainer().getTecbotSensors().getEncoder(SubsystemType.SHOOTER).getRate() );

        double rate = Robot.getRobotContainer().getTecbotSensors().getEncoder(SubsystemType.SHOOTER).getRate() / 30000;
        return rate;

    }

    public void shoot(double power) {
        Robot.getRobotContainer().getSharedMotors().setAll(power, power);
    }

    public void setShootingSpeed(ShooterPosition position) {

        switch (position) {
            case TRENCH:
                speed = TecbotConstants.SHOOTER_TRENCH_SHOOTING_SPEED;
                this.setSetpoint(speed);

                break;

            case TARGET_ZONE:
                speed = TecbotConstants.SHOOTER_TARGET_ZONE_SHOOTING_SPEED;
                this.setSetpoint(speed);

                break;

            case INITIATION_LINE:
                speed = TecbotConstants.SHOOTER_INITIATION_LINE_SHOOTING_SPEED;
                this.setSetpoint(speed);

                break;

            case OFF:
                speed = TecbotConstants.SHOOTER_OFF;
                this.setSetpoint(speed);

                break;

            default:
                DriverStation.reportError("The set speed isn´t possible", true);
        }
    }

    public void setAnglerDegrees(ShooterPosition position) {
        switch (position) {
            case TRENCH:
                angle = TecbotConstants.TRENCH_SHOOTING_ANGLE;

                break;

            case TARGET_ZONE:

                angle = TecbotConstants.TARGET_ZONE_SHOOTING_ANGLE;

                break;

            case INITIATION_LINE:
                angle = TecbotConstants.INITIATION_LINE_SHOOTING_ANGLE;
                break;

            case OFF:
                angle = TecbotConstants.SHOOTER_OFF_ANGLE;
                break;

            default:
                DriverStation.reportError("The set angle isn´t possible", true);

        }


    }

    public void setManualShooter(double manualSpeed) {
        Robot.getRobotContainer().getSharedMotors().setAll(manualSpeed, manualSpeed);
    }

    /**
     * sets the angler manually with the triggers
     *
     * @param lt Left Trigger
     * @param rt Right Trigger
     */
    public void setManualAngler(double lt, double rt) {
        double manualAngle = -lt + rt;
        anglerServo.set(manualAngle);
    }

    public enum ShooterPosition {
        TRENCH, TARGET_ZONE, INITIATION_LINE, OFF
    }

    public double getSpeed() {
        return this.speed;
    }


}