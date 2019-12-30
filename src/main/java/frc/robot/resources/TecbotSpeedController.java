/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * Add your docs here.
 *
 */

public class TecbotSpeedController {

	public enum TypeOfMotor {

		TALON_SRX, PWM_TALON_SRX, VICTOR, SPARK, JAGUAR, VICTOR_SPX, PWM_VICTOR_SPX

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

				phoenixMotor = new TalonSRX(port);

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

			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

		}

	}

	public void set(double speed) {
		speed *= inverted ? -1 : 1;

		switch (motorToUse) {

			case TALON_SRX:
				//SmartDashboard.putNumber("Motor " + this, phoenixMotor.getMotorOutputPercent());
				phoenixMotor.set(ControlMode.PercentOutput, speed);

				break;

			case PWM_TALON_SRX:

				frcMotor.set(speed);

				break;

			case VICTOR:

				frcMotor.set(speed);

				break;

			case SPARK:

				frcMotor.set(speed);

				break;

			case JAGUAR:

				frcMotor.set(speed);

				break;

			case VICTOR_SPX:

				phoenixMotor.set(ControlMode.PercentOutput, speed);

				break;

			case PWM_VICTOR_SPX:

				frcMotor.set(speed);

				break;

			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

		}

	}

	public int getEncPosition() {

		switch (motorToUse) {

			case TALON_SRX:

				return phoenixMotor.getSelectedSensorPosition(0);

			case PWM_TALON_SRX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			case VICTOR:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			case SPARK:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			case JAGUAR:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			case VICTOR_SPX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			case PWM_VICTOR_SPX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				return 0;

			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

				return 0;

		}

	}

	public void stopMotor() {

		switch (motorToUse) {

			case TALON_SRX:

				phoenixMotor.set(ControlMode.PercentOutput, 0);

				break;

			case PWM_TALON_SRX:

				frcMotor.stopMotor();

				break;

			case VICTOR:

				frcMotor.stopMotor();

				break;

			case SPARK:

				frcMotor.stopMotor();

				break;

			case JAGUAR:

				frcMotor.stopMotor();

				break;

			case VICTOR_SPX:

				phoenixMotor.set(ControlMode.PercentOutput, 0);

				break;

			case PWM_VICTOR_SPX:

				frcMotor.stopMotor();

				break;



			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

				break;

		}

	}

	public double get() {

		switch (motorToUse) {

			case TALON_SRX:

				return phoenixMotor.getMotorOutputPercent();

			case PWM_TALON_SRX:

				return frcMotor.get();

			case VICTOR:

				return frcMotor.get();

			case SPARK:

				return frcMotor.get();

			case JAGUAR:

				return frcMotor.get();

			case VICTOR_SPX:

				return phoenixMotor.getMotorOutputPercent();

			case PWM_VICTOR_SPX:

				return frcMotor.get();

			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

				return 0.0;

		}

	}

	public void setEncoderPosition(int value) {

		switch (motorToUse) {

			case TALON_SRX:

				phoenixMotor.setSelectedSensorPosition(value);

				break;

			case PWM_TALON_SRX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			case VICTOR:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			case SPARK:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			case JAGUAR:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			case VICTOR_SPX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			case PWM_VICTOR_SPX:

				DriverStation.reportWarning("That is not a Talon SRX!", true);

				break;

			default:

				DriverStation.reportError("That type of motor doesn't exist!", true);

				break;

		}

	}

	public TalonSRX getTalonSRX() {
		if (motorToUse == TypeOfMotor.TALON_SRX)
			return (TalonSRX) phoenixMotor;
		return null;

	}

	/*
	 * WARNING: this will only work with TALON SRX
	 *
	 */
	public void setBrakeMode(boolean doBrake) {

		((TalonSRX) phoenixMotor).setNeutralMode(doBrake ? NeutralMode.Brake : NeutralMode.Coast);

	}

	public void setInverted(boolean i){
		inverted = i;
	}
	public boolean isInverted(){
		return inverted;
	}

}