/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.*;

/**
 * TecbotEncoder can be used to create almost any kind of encoder,
 * either linked to a TALON_SRX {@link TecbotSpeedController.TypeOfMotor}
 * or directly connected to the RoboRIO.
 */

public class TecbotEncoder implements CounterBase {

    private Encoder encoder;

    private TecbotSpeedController talonEncoder;

    boolean isInverted;

    private double talonDistancePerPulse = 1;

    public TecbotEncoder(Encoder encoder) {

        this.encoder = encoder;

    }

    public TecbotEncoder(TecbotSpeedController talonEncoder) {

        this.talonEncoder = talonEncoder;
        this.doDefaultSRXConfig();
    }

    public TecbotEncoder(DigitalSource a, DigitalSource b) {

        this.encoder = new Encoder(a, b);

    }

    public TecbotEncoder(int a, int b) {

        this.encoder = new Encoder(a, b);

    }

    public void setDistancePerPulse(double distancePerPulse) {

        if (encoder != null) {

            encoder.setDistancePerPulse(distancePerPulse);

        } else if (talonEncoder != null) {

            talonDistancePerPulse = distancePerPulse;

        }

    }

    public double getDistance() {

        if (encoder != null) {

            return encoder.getDistance();

        }

        if (talonEncoder != null) {

            return talonEncoder.getEncPosition() * talonDistancePerPulse;

        }

        DriverStation.reportWarning("Theres no encoder instantiated", true);

        return 0.0;

    }

    public double getRate() {

        if (encoder != null) {

            return encoder.getRate();

        }

        if (talonEncoder != null) {

            return talonEncoder.getTalonSRX().getSelectedSensorVelocity();

        }

        DriverStation.reportWarning("There's no encoder instantiated", true);

        return 0.0;

    }

    public int getRaw() {

        if (encoder != null) {

            return encoder.getRaw();

        }

        if (talonEncoder != null) {

            return (isInverted ? -1 : 1) * talonEncoder.getEncPosition();

        }

        DriverStation.reportWarning("There's no encoder instantiated", true);

        return 0;

    }
    public double getSparkRaw(){
        if(talonEncoder != null)
            return (isInverted() ? -1:1) *talonEncoder.getSparkEncPosition();
        else
            return 0;
    }

    public void setEncoderPosition(int value) {
        talonEncoder.setEncoderPosition(0);
    }

    public void resetEncoder() {

        if (encoder != null) {
            encoder.reset();
        }

        if (talonEncoder != null) {

            this.setEncoderPosition(0);
        }
    }


    @Override
    public int get() {
        if (encoder != null)
            return encoder.get();

        return 0;
    }

    @Override
    public void reset() {
        this.resetEncoder();
    }

    @Override
    public double getPeriod() {
        if (encoder != null)
            return encoder.getPeriod();

        return 0;
    }

    @Override
    public void setMaxPeriod(double maxPeriod) {
        if (encoder != null)
            encoder.setMaxPeriod(maxPeriod);

    }

    @Override
    public boolean getStopped() {
        if (encoder != null)
            return encoder.getStopped();

        return false;
    }

    @Override
    public boolean getDirection() {
        if (encoder != null)
            return encoder.getDirection();

        return false;
    }

    public void doDefaultSRXConfig() {
        if(talonEncoder.getType() == TecbotSpeedController.TypeOfMotor.TALON_SRX) {
            talonEncoder.getTalonSRX().configFactoryDefault();
            talonEncoder.getTalonSRX().setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
            talonEncoder.getTalonSRX().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        }
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public boolean isInverted() {
        return isInverted;
    }

}