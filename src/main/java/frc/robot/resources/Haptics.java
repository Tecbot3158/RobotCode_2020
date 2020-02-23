package frc.robot.resources;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;

public class Haptics {

    private Joystick pilot;

    private Notifier starter, finisher;

    private double intensity;
    private double duration;

    private GenericHID.RumbleType rumbleType;
    private boolean activateBoth;

    /**
     * @param pilot The joystick to which the haptics will be applied.
     */
    public Haptics(Joystick pilot) {
        this.pilot = pilot;
        starter = new Notifier(this::setRumble);
        finisher = new Notifier(this::stopRumble);

    }

    /**
     * Periodic rumble of a given intensity, period and duration.
     *
     * @param intensity  The intensity of the rumble.
     * @param period     The time between the starting of the rumble.
     * @param duration   The duration of the rumble.
     * @param rumbleType The rumble type, left or right.
     */
    public void startPeriodicRumble(double intensity, double period, double duration, GenericHID.RumbleType rumbleType) {
        this.intensity = intensity;
        this.rumbleType = rumbleType;
        this.duration = duration;
        starter.startPeriodic(period);
        activateBoth = false;
    }

    private void setRumble() {
        if (activateBoth) {
            pilot.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
            pilot.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
        } else {
            pilot.setRumble(rumbleType, intensity);
        }

        finisher.startSingle(duration);
    }

    /**
     * Periodic rumble of a given intensity, period and duration.
     * It will be applied to both sides.
     *
     * @param intensity The intensity of the rumble.
     * @param period    The time between the starting of the rumble.
     * @param duration  The duration of the rumble.
     */
    public void startPeriodicRumble(double intensity, double period, double duration) {
        this.intensity = intensity;
        this.duration = duration;
        starter.startPeriodic(period);
        activateBoth = true;
    }

    private void stopRumble() {
        if (activateBoth) {
            pilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            pilot.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        } else
            pilot.setRumble(rumbleType, 0);
    }

    /**
     * Single rumble of a given intensity and duration.
     *
     * @param intensity The intensity of the rumble.
     * @param duration  The duration of the rumble.
     */
    public void rumble(double intensity, double duration) {
        pilot.setRumble(GenericHID.RumbleType.kLeftRumble, intensity);
        pilot.setRumble(GenericHID.RumbleType.kRightRumble, intensity);
        activateBoth = true;
        finisher.startSingle(duration);
    }

    /**
     * Single rumble of a given intensity and duration.
     *
     * @param intensity  The intensity of the rumble.
     * @param duration   The duration of the rumble.
     * @param rumbleType The rumble type, left or right.
     */
    public void rumble(double intensity, double duration, GenericHID.RumbleType rumbleType) {
        pilot.setRumble(rumbleType, intensity);
        finisher.startSingle(duration);
        this.rumbleType = rumbleType;
        activateBoth = false;
    }
}
