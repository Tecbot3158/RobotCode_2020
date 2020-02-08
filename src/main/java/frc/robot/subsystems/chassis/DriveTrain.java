/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.chassis;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.*;

public class DriveTrain extends SubsystemBase {
    // Motors
    TecbotMotorList leftMotors;
    TecbotMotorList rightMotors;
    TecbotMotorList middleMotors;

    DoubleSolenoid dragonFlyWheelSolenoid;

    DoubleSolenoid transmission;

    public enum TransmissionMode {
        torque, speed
    }

    TransmissionMode transmissionState;

    // If driving with inverted axes;
    boolean reverse = false;

    boolean arrivedToThePosition = false;
    double target, diffPos, diffAng;

    // Mecanum and Swerve move require the robot to stay in the same angle (unless
    // turning) so hasSetAngle
    // checks if the angle has been set.
    boolean hasSetAngle;
    // The angle the robot will stay in during mecanum or swerve drive unless
    // turning.
    double startingAngle;

    public enum DrivingMode {
        Default, Pivot, Mecanum, Swerve
    };

    private DrivingMode currentDrivingMode = DrivingMode.Default;

    public DrivingMode getCurrentDrivingMode() {
        return currentDrivingMode;
    }

    public enum Side {
        RIGHT, LEFT;
    }

    // The desired angle when using PID
    public double pidAngleTarget = 0;

    // The desired encoder count when using PID
    public double pidStraightTarget = 0;

    public DriveTrain() {

        transmission = new DoubleSolenoid(RobotMap.TRANSMISSION_PORT[0], RobotMap.TRANSMISSION_PORT[1]);

        if(RobotMap.DRAGON_FLY_IS_AVAILABLE)
        dragonFlyWheelSolenoid = new DoubleSolenoid(RobotMap.WHEEL_SOLENOID_PORTS[0], RobotMap.WHEEL_SOLENOID_PORTS[1]);


        if (RobotMap.LEFT_CHASSIS_PORTS.length != RobotMap.RIGHT_CHASSIS_PORTS.length)
            DriverStation.reportError("More motors in one side.", true);

      leftMotors = RobotConfigurator.buildMotorList(RobotMap.LEFT_CHASSIS_PORTS,
              RobotMap.LEFT_CHASSIS_INVERTED_MOTORS, RobotMap.LEFT_CHASSIS_MOTOR_TYPES);

      rightMotors = RobotConfigurator.buildMotorList(RobotMap.RIGHT_CHASSIS_PORTS,
              RobotMap.RIGHT_CHASSIS_INVERTED_MOTORS, RobotMap.RIGHT_CHASSIS_MOTOR_TYPES);

      middleMotors = RobotConfigurator.buildMotorList(RobotMap.MIDDLE_WHEEL_PORTS,
              RobotMap.MIDDLE_WHEEL_INVERTED_MOTORS, RobotMap.MIDDLE_WHEEL_MOTOR_TYPES);
    }

    /**
     * The default driving method for all driving modes.
     *
     * @param x           The value of the x axis
     * @param y           The value of the y axis
     * @param turn        The value of the axis designated for turing
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void defaultDrive(double x, double y, double turn, double middleWheel) {

        switch (currentDrivingMode) {
            case Default:
                dragonFlyDrive(x,reverse?-1:1 *  y, middleWheel);
                break;
            case Pivot:
                pivot(x, reverse?-1:1 * y);
                break;
            case Mecanum:
                mecanumDrive(reverse?-1:1 * x, reverse?-1:1 * y, turn);
                break;
            case Swerve:
                swerveMove(x, y, turn);
                break;
            default:
                DriverStation.reportError("Driving mode not recognized", true);
        }

    }

    public void driveSide( Side side, double power) {
        switch (side) {
            case LEFT:
                leftMotors.setAll(power);
                break;
            case RIGHT:
                rightMotors.setAll(power);
                break;

        }
    }

    public void setMiddleWheel(double power){
        middleMotors.setAll(power);
    }

    public void tankDrive(double leftPower, double rightPower) {
        driveSide(Side.LEFT, leftPower);
        driveSide(Side.RIGHT, rightPower);
    }

    public void drive(double turn, double speed) {

        double leftPower = (turn + speed);
        double rightPower = -turn + speed;

        tankDrive(leftPower,rightPower);
    }

    public boolean turn(double target, double maxPower) {

        maxPower = Math.clamp(maxPower, 0, 1);

        double diffAngle = Robot.getRobotContainer().getTecbotSensors().getYaw() - target;

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
        dragonFlyDrive(0, 0, 0);
    }

    /**
     * Rises or lowers the wheel.
     *
     * @param state The desired state for the wheel, true for rising.
     */

    public void setDragonFlyWheelState(boolean state) {
        if (state) {
            dragonFlyWheelSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            dragonFlyWheelSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean getDragonFlyWheelState() {
        return (dragonFlyWheelSolenoid.get() == DoubleSolenoid.Value.kForward);
    }

    /**
     * The default driving method for driving a DragonFly chassis manually.
     *
     * @param turn        The value of the joystick used for turning.
     * @param speed       The value of the joystick used for moving straight.
     * @param middleWheel The value that will be given to the middle wheel
     */
    public void dragonFlyDrive(double turn, double speed, double middleWheel) {
        setMiddleWheel(middleWheel);
        drive(speed, turn);
    }

    /**
     * Moves the robot pivoting in left or right wheels
     *
     * @param turn  The value of the joystick used for turning.
     * @param speed The value of the joystick used for moving straight.
     */
    public void pivot(double turn, double speed) {
        if (turn <= 0) {
            tankDrive(-.1, speed);
        } else {
            tankDrive(speed, -.1);
        }
    }

    /**
     * This method controls the robot as if it were a mecanum chassis. <br>
     * <strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param x    The desired movement in x axis, from -1 to 1.
     * @param y    The desired movement in the y axis, from -1 to 1.
     * @param turn The desired turn that the robot will have while driving, from -1
     *             to 1.
     */
    public void mecanumDrive(double x, double y, double turn) {

        if (!hasSetAngle) {
            startingAngle = Robot.getRobotContainer().getTecbotSensors().getYaw();
            hasSetAngle = true;

            // This condition will happen once every time the robot enters mecanum drive.
            // Mecanum drive needs to be lowered. We need to lower the wheel once the robot
            // enters mecanum drive.
            setDragonFlyWheelState(false);
        }
        if (turn >= .1 || turn <= -.1)
            startingAngle = Robot.getRobotContainer().getTecbotSensors().getYaw();

        double deltaAngle = Robot.getRobotContainer().getTecbotSensors().getYaw() - startingAngle;

        // Prevents robot from turning in the incorrect direction
        if (deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        } else if (deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }
        double correction = TecbotConstants.TURN_CORRECTION * deltaAngle;

        double leftSide = TecbotConstants.MIDDLE_SIDES_CORRECTION * (y - correction + turn);
        double rightSide = TecbotConstants.MIDDLE_SIDES_CORRECTION * (y + correction - turn);

        tankDrive(leftSide, rightSide);
        setMiddleWheel(x);

    }

    /**
     * This method takes an angle and makes the robot move in that direction using
     * the middle wheel. It can also turn the robot while moving. <br>
     * <strong>No field orientated drive is implemented in this method.</strong>
     *
     * @param angle    The angle in degrees relative to the robot at which the robot
     *                 will move.
     * @param maxPower The max power that will be given to the motors.
     * @param turn     The desired turn that the robot will have while driving, from
     *                 -1 to 1.
     */
    public void driveToAngle(double angle, double maxPower, double turn) {
        double x = Math.sin(Math.toRadians(angle)) * maxPower;
        double y = Math.cos(Math.toRadians(angle)) * maxPower;

        mecanumDrive(x, y, turn);
    }

    /**
     * This method uses field orientated drive to make the robot move a certain
     * value in x and a certain value in y while turning.
     *
     * @param x    The desired movement that the robot will have in the x axis
     * @param y    The desired movement that the robot will have in the y axis
     * @param turn The desired turn that the robot will have while driving, from -1
     *             to 1.
     */
    public void swerveMove(double x, double y, double turn) {
        // The angle relative to the field given by the x and the y
        double absoluteAngle = 0;
        if (y != 0) {
            absoluteAngle = Math.toDegrees(Math.atan(x / y));
            if (x > 0)
                absoluteAngle = 90;
            if (x < 0)
                absoluteAngle = -90;
        }
        if (y < 0) {
            if (x < 0) {
                absoluteAngle -= 180;
            } else {
                absoluteAngle += 180;
            }
        }
        // The angle at which the robot will move, considering its rotation.
        double relativeAngle = absoluteAngle - Robot.getRobotContainer().getTecbotSensors().getYaw();
        // The max power that will be given to the motors.
        double speed = Math.sqrt((x * x) + (y * y));

        driveToAngle(relativeAngle, speed, turn);

    }

    public void pidTurn(double output){
        drive(output,0);
    }
    public double getPidAngleTarget(){
        return pidAngleTarget;
    }
    public void setPidAngleTarget(double target){
        pidAngleTarget = target;
    }

    public void moveStraightPID(double output){
        drive((Robot.getRobotContainer().getTecbotSensors().getYaw()-pidAngleTarget)*TecbotConstants.TURN_CORRECTION, output);
    }
    public double getPidStraightTarget(){
        return pidStraightTarget;
    }
    public void setPidStraightTarget(double target){
        pidStraightTarget = target;
    }


    public void setMecanumDrive(boolean state) {
        if (state)
            currentDrivingMode = DrivingMode.Mecanum;
        else
            currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingMecanum() {
        return (currentDrivingMode == DrivingMode.Mecanum);
    }

    public void setSwerveDrive(boolean state) {
        if (state)
            currentDrivingMode = DrivingMode.Swerve;
        else
            currentDrivingMode = DrivingMode.Default;
    }

    public boolean isMovingSwerve() {
        return (currentDrivingMode == DrivingMode.Swerve);
    }

    public void setPivoting(boolean state) {
        if (state)
            currentDrivingMode = DrivingMode.Pivot;
        else
            currentDrivingMode = DrivingMode.Default;
    }

    public boolean isPivoting() {
        return (currentDrivingMode == DrivingMode.Pivot);
    }

    public void setDefaultDrive() {
        currentDrivingMode = DrivingMode.Default;
    }

    public void setDrivingMode(DrivingMode mode) {
        currentDrivingMode = mode;
    }

    public TransmissionMode getTransmissionMode() {
        return (transmissionState);
    }


    public void setTransmissionState(TransmissionMode mode) {
        transmissionState = mode;
        if (mode == TransmissionMode.torque) {
            transmission.set(RobotMap.TORQUE_TRANSMISSION);
        } else {
            transmission.set(RobotMap.SPEED_TRANSMISSION);
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
     * 
     * @param reverse True means inverted driving.
     */

    public void setOrientation(boolean reverse) {
        this.reverse = reverse;
    }

    public boolean getOrientation() {
        return reverse;
    }

    public TecbotSpeedController getSpecificMotor(int port){
        TecbotSpeedController left = leftMotors.getSpecificMotor(port);
        TecbotSpeedController right = rightMotors.getSpecificMotor(port);
        TecbotSpeedController middle = rightMotors.getSpecificMotor(port);

        if(left != null)return left;
        else if(right != null) return right;
        else return middle;
    }

}
