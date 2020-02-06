/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;
import frc.robot.splines.CubicSpline;
import frc.robot.splines.Derivative;
import frc.robot.splines.SplineGenerator;

public class SplineMove extends CommandBase {
    /**
     * Creates a new Command.
     */

    // ALL UNITS ARE IN METERS

    double targetPower;
    double wheelRadius = TecbotConstants.K_CHASSIS_WHEEL_DIAMETER / 2;
    double robotRadius = TecbotConstants.K_DISTANCE_BETWEEN_WHEELS;

    // The power to set both talons
    double leftPower, rightPower;
    // The distance traveled since last frame
    double deltaDistanceLeft = .001, deltaDistanceRight = .001, deltaMiddleWheel = .001;
    // The angle and encoder count in last frame
    double lastAngle = 0;
    double lastLeftCount = 0;
    double lastRightCount = 0;
    double lastMiddleCount = 0;
    // The distance that the wheels would travel if the robot did a 360 turn
    // The degrees that the robot has to turn in this frame
    double deltaAngle;
    // The angle that the robot has to go
    double nextAngle;
    // The x position of the robot
    float xPos = 0;
    // If robot has completed the trajectory
    boolean hasFinished = false;

    // Make the robot stop when the spline is finished.
    boolean stopWhenFinished;

    // The math used to work the splines supposes that x0<x1<x2, in other words, that the points are ordinated
    // going right, for going left, we need to invert some things, so this boolean controls those things
    boolean goingLeft;

    // At the beginning of the spline, full power will be given to the motors, but after this point
    // Speed will start to reduce gradually in order to improve accuracy. This point will be an x point,
    // when the robot reaches that x point, speed will start to reduce.
    double startReducingSpeedPoint;

    // At some point in the spline, the robot might start using swerve drive in order to continue with the spline
    // While its turning to align with a certain target angle.
    double startSwervingPoint;
    // After having started using swerve drive, the robot will try to reach this angle
    double expectedFinalAngle;

    Notifier not;

    // The points where the robot will pass, x0 and y0 are the initial position
    double x1;
    double x2;
    double z1;
    double z2;
    float x0 = 0, z0 = 0;
    // Both intervals' trajectory
    CubicSpline spline0, spline1;
    // Derivative of both splines
    Derivative derivada0, derivada1;


    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = true;
        // Set to -1 means no speed reduction.
        startReducingSpeedPoint  =  -1;
        // Set to -1 means it will never move in swerve mode
        startSwervingPoint = -1;
        expectedFinalAngle = -1;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, boolean g_stopWhenFinished) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = g_stopWhenFinished;
        // Set to -1 means no speed reduction.
        startReducingSpeedPoint  =  -1;
        // Set to -1 means it will never move in swerve mode
        startSwervingPoint = -1;
        expectedFinalAngle = -1;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, double g_startReducingSpeedPoint) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = true;
        startReducingSpeedPoint = Math.abs(g_startReducingSpeedPoint);
        // Set to -1 means it will never move in swerve mode
        startSwervingPoint = -1;
        expectedFinalAngle = -1;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, boolean g_stopWhenFinished, double g_startReducingSpeedPoint) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = g_stopWhenFinished;
        startReducingSpeedPoint = Math.abs(g_startReducingSpeedPoint);
        // Set to -1 means it will never move in swerve mode
        startSwervingPoint = -1;
        expectedFinalAngle = -1;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, double g_startSwervingPoint, double g_expectedFinalAngle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = true;
        // Set to -1 means no speed reduction.
        startReducingSpeedPoint  =  -1;
        startSwervingPoint = g_startSwervingPoint;
        expectedFinalAngle = g_expectedFinalAngle;

    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, boolean g_stopWhenFinished , double g_startSwervingPoint, double g_expectedFinalAngle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = g_stopWhenFinished;
        // Set to -1 means no speed reduction.
        startReducingSpeedPoint  =  -1;
        startSwervingPoint = g_startSwervingPoint;
        expectedFinalAngle = g_expectedFinalAngle;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, double g_startReducingSpeedPoint, double g_startSwervingPoint, double g_expectedFinalAngle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = true;
        startReducingSpeedPoint = Math.abs(g_startReducingSpeedPoint);
        startSwervingPoint = g_startSwervingPoint;
        expectedFinalAngle = g_expectedFinalAngle;
    }
    public SplineMove(double g_x1, double g_x2, double g_z1, double g_z2, double g_targetPower, boolean g_stopWhenFinished, double g_startReducingSpeedPoint, double g_startSwervingPoint, double g_expectedFinalAngle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());

        x1 = g_x1;
        x2 = g_x2;
        z1 = g_z1;
        z2 = g_z2;
        targetPower = g_targetPower;

        spline0 = SplineGenerator.GenerateSpline0(x0, x1, x2, z0, z1, z2);
        spline1 = SplineGenerator.GenerateSpline1(x0, x1, x2, z0, z1, z2);

        derivada0 = SplineGenerator.DerivateSpline(spline0);
        derivada1 = SplineGenerator.DerivateSpline(spline1);

        if(x1 < 0)
            goingLeft = true;

        stopWhenFinished = g_stopWhenFinished;
        startReducingSpeedPoint = Math.abs(g_startReducingSpeedPoint);
        startSwervingPoint = g_startSwervingPoint;
        expectedFinalAngle = g_expectedFinalAngle;
    }


    @Override
    public void initialize() {

        xPos = 0;
        lastAngle = (float) Robot.getRobotContainer().getTecbotSensors().getYaw();

        not = new Notifier(this::move);
        not.startPeriodic(.05f);

        // The math used to work thr splines out needs the x to be ordinated, either to right or left.
        // If this isn't the case, then the spline won't be valid
        if(Math.abs(x2) < Math.abs(x1))
            DriverStation.reportError("Not a valid spline", true);

    }


    private void move() {

        System.out.println("left encoder: " + Robot.getRobotContainer().getDriveTrain().getLeftEncoder().getRaw());
        System.out.println("right encoder: " + Robot.getRobotContainer().getDriveTrain().getRightEncoder().getRaw());

        //The distance that each wheel traveled
        // deltaDistanceLeft has the "-" because when right increases, left decreases (because of their positions in the robot)
        // In other words, they're physically inverted
        deltaDistanceLeft = -(-lastLeftCount * TecbotConstants.K_CHASSIS_ENCODER_TO_METERS) + (Robot.getRobotContainer().getDriveTrain().getLeftPosition() * TecbotConstants.K_CHASSIS_ENCODER_TO_METERS);
        deltaDistanceRight = (-lastRightCount * TecbotConstants.K_CHASSIS_ENCODER_TO_METERS) + (Robot.getRobotContainer().getDriveTrain().getRightPosition() * TecbotConstants.K_CHASSIS_ENCODER_TO_METERS);

        deltaMiddleWheel = (-lastMiddleCount * TecbotConstants.K_MIDDLE_WHEEL_ENCODER_TO_METERS) + (Robot.getRobotContainer().getDriveTrain().getMiddlePosition() * TecbotConstants.K_MIDDLE_WHEEL_ENCODER_TO_METERS);

        // If the spline is going left, then we need to trick the math to think we're going right while the robot
        // is making the exact opposite.
        if(goingLeft){
            double bubble = deltaDistanceLeft;
            deltaDistanceLeft = deltaDistanceRight;
            deltaDistanceRight = bubble;
        }

        double leftSpeed = deltaDistanceLeft;
        double rightSpeed = deltaDistanceRight;

        double currentAngle = goingLeft ? -1 : 1 * TecbotSensors.getYaw();

        // When moving tank, ICC kinematics are needed to calculate the position of the robot
        // When moving swerve, since we're using polar coordinates, regular trigonometry is used to calculate it.
        if(xPos < startSwervingPoint) {
            //The distance from the middle point between the wheels to the Instant Curvature Center
            double distanceToICC = 0;
            if (rightSpeed != leftSpeed) {
                distanceToICC = ((leftSpeed + rightSpeed) / (rightSpeed - leftSpeed));
                //Calculates de xPos based on the ICC and the delta angle
                xPos -= (distanceToICC * (Math.cos(Math.toRadians(lastAngle)) - Math.cos(Math.toRadians(currentAngle)))) / 2;
                System.out.println("ICC" + distanceToICC);
            } else {
                // If going straight, ICC becomes infinite, meaning the robot is moving in a straight line
                // Basic trigonometry will be applied in that case
                xPos += Math.cos(Math.toRadians(currentAngle)) * deltaDistanceLeft;
            }

            System.out.println("xPos " + xPos);
        }else {
            double averageY = (deltaDistanceLeft + deltaDistanceRight) / 2;
            double totalDistance = Math.hypot(deltaMiddleWheel, averageY);
            // The angle at which the robot moved, relative to the robot
            double relativeAngle = Math.atan(deltaMiddleWheel/averageY);
            // The angle at which the robot moved, relative to the field
            double absoluteAngle = relativeAngle + Math.toRadians(TecbotSensors.getYaw());
            xPos+= Math.sin(absoluteAngle) * totalDistance;
        }
        //If finished
        if (xPos >= x2) {
            if(stopWhenFinished){
                Robot.getRobotContainer().getDriveTrain().stop();
            }

            hasFinished = true;
            not.stop();
        }

        // Decides if the robot is moving along the first or the second spline
        if (xPos <= x1) {
            nextAngle = SplineGenerator.angleFromDerivate(derivada0, xPos);
        } else if (xPos <= x2 && xPos >= x1) {
            nextAngle = SplineGenerator.angleFromDerivate(derivada1, xPos);
        } else {
            System.out.println("finished");
        }

        nextAngle *= goingLeft ? -1 : 1;

        deltaAngle =  (nextAngle - TecbotSensors.getYaw());
        System.out.println("Target angle " + nextAngle);

        //Prevents robot from turning in the incorrect direction
        if(deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        }
        else if(deltaAngle < -180) {
            deltaAngle = -deltaAngle + 360;
        }


        double axis =  frc.robot.resources.Math.clamp(deltaAngle / 20, -1, 1);
        double power = targetPower;
        if(xPos > startReducingSpeedPoint){

            double currentY;
            if(xPos < x1)
                currentY = Math.pow(xPos , 3) * spline0.cubicX + Math.pow(xPos , 2) * spline0.squareX + xPos * spline0.x + spline0.indep;
            else
                currentY = Math.pow(xPos , 3) * spline1.cubicX + Math.pow(xPos , 2) * spline1.squareX + xPos * spline1.x + spline1.indep;

            double targetY = Math.pow(x2 , 3) * spline1.cubicX + Math.pow(x2 , 2) * spline1.squareX + x2 * spline1.x + spline1.indep;

            double linearDifference = Math.hypot(x2-xPos, targetY-currentY);
            power = linearDifference * TecbotConstants.SPLINE_REDUCING_SPEED_CONSTANT * targetPower;
        }

        if(xPos < startSwervingPoint)
            Robot.getRobotContainer().getDriveTrain().drive(power, -axis);
        else{
            double deltaFinalAngle = expectedFinalAngle - TecbotSensors.getYaw();
            //Prevents robot from turning in the incorrect direction
            if(deltaFinalAngle > 180) {
                deltaFinalAngle = deltaFinalAngle - 360;
            }
            else if(deltaFinalAngle < -180) {
                deltaFinalAngle = -deltaFinalAngle + 360;
            }
            double turn = deltaFinalAngle * TecbotConstants.SPLINE_TURN_CORRECTION;
            // The X and Y relative to the field based on the angle
            double fieldX = Math.sin(Math.toRadians(nextAngle));
            double fieldY = Math.cos(Math.toRadians(nextAngle));
            Robot.getRobotContainer().getDriveTrain().swerveMove(fieldX,fieldY, turn);
        }

        System.out.println("clamped value"+ axis);
        System.out.println("delta"+ deltaAngle);

        lastLeftCount = Robot.getRobotContainer().getDriveTrain().getLeftPosition();
        lastRightCount = Robot.getRobotContainer().getDriveTrain().getRightPosition();
        lastMiddleCount = Robot.getRobotContainer().getDriveTrain().getMiddlePosition();

        // For some reason that I don't really understand, when going right (normal spline)
        // Last angle must be inverted in order to work.
        lastAngle = goingLeft ? 1 : -1 * TecbotSensors.getYaw();

        SmartDashboard.putNumber("Correction Value", axis);
        SmartDashboard.putNumber("Delta Angle", deltaAngle);
    }



    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        not.stop();
    }

    @Override
    public boolean isFinished() {
        return hasFinished;
    }
}
