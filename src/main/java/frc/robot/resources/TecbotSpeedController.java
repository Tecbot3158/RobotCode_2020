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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PWMVictorSPX;

/**
 *
 * Add your docs here.
 *
 */

public class TecbotSpeedController {

	public enum TypeOfMotor {

		TALON_SRX, PWM_TALON_SRX, VICTOR, SPARK, ,CAN_SPARK_BRUSHLESS ,CAN_SPARK_BRUSHED, JAGUAR, VICTOR_SPX, PWM_VICTOR_SPX

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

		if(phoenixMotor != null) phoenixMotor.set(ControlMode.PercentOutput, speed);
		if(frcMotor != null) frcMotor.set( speed);

		if(phoenixMotor == null && frcMotor == null)
			DriverStation.reportError("That type of motor doesn't exist!", true);
	}

	public int getEncPosition() {

		if(motorToUse ==  TypeOfMotor.TALON_SRX)
			return phoenixMotor.getSelectedSensorPosition(0);
		else if(motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS){
			return (int) ((CANSparkMax)frcMotor).getEncoder().getPosition();
		}else
			DriverStation.reportWarning("That is not a Talon SRX nor a Spark Max!", true);
		return 0;

	}

	public void stopMotor() {

		if(frcMotor != null) frcMotor.stopMotor();
		if(phoenixMotor != null) phoenixMotor.set(ControlMode.PercentOutput,0);

	}

	public double get() {

		if(phoenixMotor != null) return phoenixMotor.getMotorOutputPercent();
		if(frcMotor != null) return frcMotor.get();
		else
			DriverStation.reportError("Null motor", true);
		return 0;

	}

	public void setEncoderPosition(int value) {

		if(motorToUse == TypeOfMotor.TALON_SRX) phoenixMotor.setSelectedSensorPosition(value);
		else DriverStation.reportWarning("Not a talonSRX, sensor position not updated", false);

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