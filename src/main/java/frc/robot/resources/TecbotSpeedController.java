/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;

/**
 * Creates new TecbotSpeedController and allows a generalized
 * use of speed controllers. To see the currently supported
 * motor controllers you can check {@link TypeOfMotor}
 */
public class TecbotSpeedController {


    public enum TypeOfMotor {

        TALON_SRX, PWM_TALON_SRX, VICTOR, SPARK, CAN_SPARK_BRUSHLESS, CAN_SPARK_BRUSHED, JAGUAR, VICTOR_SPX, PWM_VICTOR_SPX

    }

    private BaseMotorController phoenixMotor;

    private SpeedController frcMotor;

    private TypeOfMotor motorToUse;

    public TypeOfMotor getType() {
        return motorToUse;
    }

    boolean inverted;

    public TecbotSpeedController(int port, TypeOfMotor m) {

        motorToUse = m;


        switch (motorToUse) {

            case TALON_SRX:

                phoenixMotor = new WPI_TalonSRX(port);

                phoenixMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

                break;

            case PWM_TALON_SRX:

                frcMotor = new PWMTalonSRX(port);

                break;

            case VICTOR:

                frcMotor = new Victor(port);

                break;

            case SPARK:

                frcMotor = new Spark(port);

                break;

            case JAGUAR:

                frcMotor = new Jaguar(port);

                break;

            case VICTOR_SPX:

                phoenixMotor = new VictorSPX(port);

                break;

            case PWM_VICTOR_SPX:

                frcMotor = new PWMVictorSPX(port);

                break;
            case CAN_SPARK_BRUSHLESS:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

                break;
            case CAN_SPARK_BRUSHED:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushed);

                break;
            default:

                DriverStation.reportError("That type of motor doesn't exist!", true);

        }

    }

    public void set(double speed) {
        speed *= inverted ? -1 : 1;

        if (phoenixMotor != null) phoenixMotor.set(ControlMode.PercentOutput, speed);
        if (frcMotor != null) frcMotor.set(speed);

        if (phoenixMotor == null && frcMotor == null)
            DriverStation.reportError("That type of motor doesn't exist!", true);
    }

    public int getEncPosition() {

        if (motorToUse == TypeOfMotor.TALON_SRX)
            return phoenixMotor.getSelectedSensorPosition(0);
        else
            DriverStation.reportWarning("That is not a Talon SRX nor a Spark Max!", true);
        return 0;

    }

    public double getSparkEncPosition() {
        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS) {
            return ((CANSparkMax) frcMotor).getEncoder().getPosition();
        } else
            return 0;
    }

    public void stopMotor() {

        if (frcMotor != null) frcMotor.stopMotor();
        if (phoenixMotor != null) phoenixMotor.set(ControlMode.PercentOutput, 0);

    }

    public double get() {

        if (phoenixMotor != null) return phoenixMotor.getMotorOutputPercent();
        if (frcMotor != null) return frcMotor.get();
        else
            DriverStation.reportError("Null motor", true);
        return 0;

    }

    public void setEncoderPosition(int value) {

        if (motorToUse == TypeOfMotor.TALON_SRX) phoenixMotor.setSelectedSensorPosition(value);
        else DriverStation.reportWarning("Not a talonSRX, sensor position not updated", false);

    }

    public WPI_TalonSRX getTalonSRX() {
        if (motorToUse == TypeOfMotor.TALON_SRX)
            return (WPI_TalonSRX) phoenixMotor;
        return null;

    }

    public CANSparkMax getCANSparkMax() {
        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS || motorToUse == TypeOfMotor.CAN_SPARK_BRUSHED) {
            return (CANSparkMax) frcMotor;
        }
        return null;
    }

    /*
     * WARNING: this will only work with TALON SRX and Spark Max
     *
     */
    public void setBrakeMode(boolean doBrake) {

        if (phoenixMotor != null)
            ((WPI_TalonSRX) phoenixMotor).setNeutralMode(doBrake ? NeutralMode.Brake : NeutralMode.Coast);
        if (frcMotor != null)
            ((CANSparkMax) frcMotor).setIdleMode(doBrake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    }

    public void setInverted(boolean i) {
        inverted = i;
    }

    public boolean isInverted() {
        return inverted;
    }

}