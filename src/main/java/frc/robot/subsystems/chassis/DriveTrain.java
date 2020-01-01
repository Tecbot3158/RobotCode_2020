/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.*;
import frc.robot.resources.Math;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

public class DriveTrain extends Subsystem {

    //Motors
    List<TecbotSpeedController> leftMotors;
    List<TecbotSpeedController> rightMotors;
    TecbotSpeedController middle;
    DifferentialDrive drive;

    DoubleSolenoid wheelSolenoid;

    DoubleSolenoid transmission;
    boolean transmissionOn = false;


    public enum TransmissionMode {torque, speed}

    TransmissionMode transmissionState;

    // If driving with inverted axes;
    boolean reverse = false;

    boolean arrivedToThePosition = false;
    double target, diffPos, diffAng;

    TecbotEncoder leftEncoder, rightEncoder, wheelEncoder;

    // Mecanum and Swerve move require the robot to stay in the same angle (unless turning) so hasSetAngle
    // checks if the angle has been set.
    boolean hasSetAngle;
    // The angle the robot will stay in during mecanum or swerve drive unless turning.
    double startingAngle;

    public enum DrivingMode {Default, Pivot,  Mecanum, Swerve};

    private DrivingMode currentDrivingMode = DrivingMode.Default;
    public DrivingMode getCurrentDrivingMode() {
        return currentDrivingMode;
    }

    public enum Side {
        RIGHT, LEFT;
    }

    public DriveTrain() {

        transmission = new DoubleSolenoid(RobotMap.transmission_port_1, RobotMap.transmission_port_2);
        wheelSolenoid = new DoubleSolenoid(RobotMap.wheelSolenoidPorts[0], RobotMap.wheelSolenoidPorts[1]);

        middle = new TecbotSpeedController(RobotMap.middleWheelPort, RobotMap.middleWheelMotorType);

        leftMotors = new ArrayList<>();
        rightMotors = new ArrayList<>();

        if (RobotMap.leftChassisPorts.length != RobotMap.rightChassisPorts.length)
            DriverStation.reportError("More motors in one side.", true);
        if (RobotMap.leftChassisPorts.length != RobotMap.leftChassisMotorTypes.length || RobotMap.rightChassisPorts.length != RobotMap.rightChassisMotorTypes.length)
            DriverStation.reportError("More ports that motor types", true);

        TecbotSpeedController leftEncoderMotor = null;
        TecbotSpeedController rightEncoderMotor = null;

        for (int i = 0; i < RobotMap.leftChassisPorts.length; i++) {
            leftMotors.add(new TecbotSpeedController(RobotMap.leftChassisPorts[i], RobotMap.leftChassisMotorTypes[i]));
            if (i == RobotMap.leftChassisMotorWithEncoder)
                leftEncoderMotor = leftMotors.get(i);
            for (int port: RobotMap.leftChassisInvertedMotors){
                if(port == RobotMap.leftChassisPorts[i])
                    leftMotors.get(i).setInverted(true);
            }
        }
        for (int i = 0; i < RobotMap.rightChassisPorts.length; i++) {
            rightMotors.add(new TecbotSpeedController(RobotMap.rightChassisPorts[i], RobotMap.rightChassisMotorTypes[i]));
            if (i == RobotMap.rightChassisMotorWithEncoder)
                rightEncoderMotor = rightMotors.get(i);
            for (int port: RobotMap.rightChassisInvertedMotors){
                if(port == RobotMap.rightChassisPorts[i])
                    rightMotors.get(i).setInverted(true);
            }
        }


        leftEncoder = RobotConfigurator.buildEncoder(leftEncoderMotor, RobotMap.leftChassisEncoderPorts[0], RobotMap.leftChassisEncoderPorts[1]);
        rightEncoder = RobotConfigurator.buildEncoder(rightEncoderMotor, RobotMap.rightChassisEncoderPorts[0], RobotMap.rightChassisEncoderPorts[1]);
        wheelEncoder = RobotConfigurator.buildEncoder(middle, RobotMap.middleWheelEncoderPorts[0], RobotMap.middleWheelEncoderPorts[0]);

        switch (leftMotors.size()) {
            case 1:
                drive = new DifferentialDrive(leftMotors.get(0), rightMotors.get(0));
                break;
            case 2:
                drive = new DifferentialDrive(leftMotors.get(0), rightMotors.get(0), leftMotors.get(1), rightMotors.get(1));
                break;
            case 3:
                drive = new DifferentialDrive(leftMotors.get(0), rightMotors.get(0), leftMotors.get(1), rightMotors.get(1), leftMotors.get(2), rightMotors.get(2));
                break;
            default:
                DriverStation.reportError("Could not instantiate differential drive", true);
                break;
        }

    }

    /**
     * The default driving method for all driving modes.
     * @param x The value of the x axis
     * @param y The value of the y axis
     * @param turn The value of the axis designated for turing
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void defaultDrive(double x, double y, double turn, double middleWheel){

        switch (currentDrivingMode){
            case Default:
                frankieDrive(x,y, middleWheel);
                break;
            case Pivot:
                pivot(x,y);
                break;
            case Mecanum:
                mecanumDrive(x,y,turn);
                break;
            case Swerve:
                swerveMove(x, y, turn);
                break;
            default:
                DriverStation.reportError("Driving mode not recognized", true);
        }

    }


    public void driveSide(@NotNull Side side, double s) {
        switch (side) {
            case LEFT:
                for (TecbotSpeedController m : leftMotors) {
                    m.set(s);
                }
                break;
            case RIGHT:
                for (TecbotSpeedController m : rightMotors) {
                    m.set(s);
                }
                break;

        }
    }

    public void tankDrive(double leftPower, double rightPower){
        driveSide(Side.LEFT, leftPower);
        driveSide(Side.RIGHT, rightPower);
    }

    public void drive(double axis, double rotationAngle) {
        if (reverse)
            drive.arcadeDrive(-axis, rotationAngle);
        else
            drive.arcadeDrive(axis, rotationAngle);
    }

    public boolean turn(double target, double maxPower) {

        maxPower = Math.clamp(maxPower, 0, 1);

        double diffAngle = Robot.tecbotgyro.getYaw() - target;

        double turnPower = Math.clamp((diffAngle / TecbotConstants.CHASSIS_TURN_MAX_DISTANCE), -maxPower, maxPower);

        double diffAbsAngle = Math.abs(diffAngle);

        SmartDashboard.putNumber("Turn Output", turnPower);
        SmartDashboard.putNumber("Difference Abs", diffAngle);

        if (diffAbsAngle >= TecbotConstants.CHASSIS_TURN_ARRIVE_OFFSET) {
            drive(0, turnPower);
        }
        if (diffAbsAngle < TecbotConstants.CHASSIS_TURN_ARRIVE_OFFSET) {
            drive(0, 0);
            return true;
        }
        return false;
    }

    public void stop() {
        frankieDrive(0,0,0);
    }

    /**
     * Rises or lowers the wheel.
     * @param state The desired state for the wheel, true for rising.
     */

    public void setWheelState(boolean state) {
        if (state) {
            wheelSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            wheelSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean getWheelState(){
        return(wheelSolenoid.get() == DoubleSolenoid.Value.kForward);
    }
    /**
     * The default driving method for driving a frankie-type chassis as a tank
     * @param turn  The value of the joystick used for turning.
     * @param speed The value of the joystick used for moving straight.
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void frankieDrive(double turn, double speed, double middleWheel){
        middle.set(middleWheel);
        drive(speed, turn);
    }

    /**
     * Moves the robot pivoting in left or right wheels
     * @param turn The value of the joystick used for turning.
     * @param speed The value of the joystick used for moving straight.
     */
    public  void pivot(double turn, double speed){
        if (turn <= 0) {
            tankDrive(-.1, speed);
        } else {
            tankDrive(speed, -.1);
        }
    }

    /**
     * This method controls the robot as if it were a mecanum chassis.
     * <br><strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param x    The desired movement in x axis, from -1 to 1.
     * @param y    The desired movement in the y axis, from -1 to 1.
     * @param turn The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void mecanumDrive(double x, double y, double turn) {

        if (!hasSetAngle) {
            startingAngle = Robot.tecbotgyro.getYaw();
            hasSetAngle = true;

            // This condition will happen once every time the robot enters mecanum drive.
            // Mecanum drive needs to be lowered. We need to lower the wheel once the robot
            // enters mecanum drive.
            setWheelState(false);
        }
        if (turn >= .1 || turn <= -.1) startingAngle = Robot.tecbotgyro.getYaw();

        double deltaAngle = Robot.tecbotgyro.getYaw() - startingAngle;

        //Prevents robot from turning in the incorrect direction
        if(deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        }
        else if(deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }
        double correction = TecbotConstants.TURN_CORRECTION * deltaAngle;

        double leftSide = TecbotConstants.MIDDLE_SIDES_CORRECTION * (y - correction + turn);
        double rightSide = TecbotConstants.MIDDLE_SIDES_CORRECTION * (y + correction - turn);

        tankDrive(leftSide, rightSide);
        middle.set(x);

    }

    /**
     * This method takes an angle and makes the robot move in that direction using the middle wheel.
     * It can also turn the robot while moving.
     * <br><strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param angle    The angle in degrees relative to the robot at which the robot will move.
     * @param maxPower The max power that will be given to the motors.
     * @param turn     The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void driveToAngle(double angle, double maxPower, double turn) {
        double x = java.lang.Math.sin(java.lang.Math.toRadians(angle)) * maxPower;
        double y = java.lang.Math.cos(java.lang.Math.toRadians(angle)) * maxPower;

        mecanumDrive(x, y, turn);
    }

    /**
     * This method uses field orientated drive to make the robot move a certain value in x and a
     * certain value in y while turning.
     *
     * @param x    The desired movement that the robot will have in the x axis
     * @param y    The desired movement that the robot will have in the y axis
     * @param turn The desired turn that the robot will have while driving, from -1 to 1.
     */
    public void swerveMove(double x, double y, double turn) {
        // The angle relative to the field given by the x and the y
        double absoluteAngle = 0;
        if (y != 0) {
            absoluteAngle = java.lang.Math.toDegrees(java.lang.Math.atan(x / y));
            if(x > 0) absoluteAngle = 90;
            if(x < 0) absoluteAngle = -90;
        }
        if (y < 0) {
            if (x < 0) {
                absoluteAngle -= 180;
            } else {
                absoluteAngle += 180;
            }
        }
        // The angle at which the robot will move, considering its rotation.
        double relativeAngle = absoluteAngle - Robot.tecbotgyro.getYaw();
        // The max power that will be given to the motors.
        double speed = java.lang.Math.sqrt((x * x) + (y * y));


        driveToAngle(relativeAngle, speed, turn);

    }


    public void setMecanumDrive(boolean state) {
        if(state) currentDrivingMode = DrivingMode.Mecanum;
        else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingMecanum() {
        return (currentDrivingMode == DrivingMode.Mecanum);
    }

    public void setSwerveDrive(boolean state) {
        if(state) currentDrivingMode = DrivingMode.Swerve;
        else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingSwerve() {
        return (currentDrivingMode == DrivingMode.Swerve);
    }

    public void setPivoting(boolean state) {
        if(state) currentDrivingMode = DrivingMode.Pivot;
        else currentDrivingMode = DrivingMode.Default;
    }

    public boolean isPivoting() {
        return (currentDrivingMode == DrivingMode.Pivot);
    }

    public void setDefaultDrive(){
        currentDrivingMode = DrivingMode.Default;
    }

    public void setDrivingMode(DrivingMode mode){
        currentDrivingMode = mode;
    }

    public TransmissionMode getTransmissionMode()
    {
        return (transmissionState);
    }
    public boolean getTransmissionState() {
        return transmissionOn;
    }

    public void setTransmissionState(TransmissionMode mode){
        transmissionState = mode;
        if(mode == TransmissionMode.torque){
            transmissionOn = true;
            transmission.set(DoubleSolenoid.Value.kForward);
        }
        else {
            transmissionOn = false;
            transmission.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Inverts the axis configuration.
     */
    public void changeOrientation() {
        reverse = !reverse;
    }

    /**
     * Changes the driving axis configuration.
     * @param reverse True means inverted driving.
     */

    public void setOrientation(boolean reverse) {
        this.reverse = reverse;
    }

    public boolean getOrientation() {
        return reverse;
    }

    public void printEncValues() {
        SmartDashboard.putNumber("Front Left DriveTrain Motor", leftEncoder.getRaw());
        SmartDashboard.putNumber("Front Right DriveTrain Motoro", rightEncoder.getRaw());
    }

    public double getLeftPosition() {
        return leftEncoder.getRaw();
    }

    public double getRightPosition() {
        return rightEncoder.getRaw();
    }

    public double getMiddlePosition(){
        return wheelEncoder.getRaw();
    }

    public TecbotEncoder getLeftEncoder() {
        return leftEncoder;
    }
    public TecbotEncoder getRightEncoder() {
        return rightEncoder;
    }
    public TecbotEncoder getWheelEncoder(){
        return wheelEncoder;
    }

    @Override
    public void initDefaultCommand() {
    }
}
