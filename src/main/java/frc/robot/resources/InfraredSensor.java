package frc.robot.resources;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InfraredSensor {

    private boolean isInverted = false;
    private AnalogInput analogInput;
    private int objectMinimumDistance;
    private boolean objectPresent = false;

    /**
     * Creates new Infrared Sensor via an {@link AnalogInput} and will act as a limitSwitch.
     * When the value is greater than a given distance, it will return true.
     * Use {@link #get()} to get the state of the Infrared Sensor.
     *
     * @param port                  channel in which the infrared analog sensor is connected.
     * @param objectMinimumDistance if the value in the analog sensor is greater than or equal to this one,
     *                              {@link #objectPresent} will become true and if it is less it will be false.
     * @param isInverted            if true, it will evaluate the analog value with <= (less than or equal to), (rather than
     *                              doing it with >= greater than or equal to) thus making it true when the value is less than
     *                              or equal to the given distance.
     */
    public InfraredSensor(int port, int objectMinimumDistance, boolean isInverted) {
        analogInput = new AnalogInput(port);
        this.objectMinimumDistance = objectMinimumDistance;
        this.isInverted = isInverted;

    }

    /**
     * Creates new Infrared Sensor via an {@link AnalogInput} and will act as a limitSwitch.
     * When the value is greater than a given distance, it will return true.
     * Use {@link #get()} to get the state of the Infrared Sensor.
     *
     * @param port                  channel in which the infrared analog sensor is connected.
     * @param objectMinimumDistance if the value in the analog sensor is greater than or equal to this one,
     *                              {@link #objectPresent} will become true and if it is less it will be false.
     */
    public InfraredSensor(int port, int objectMinimumDistance) {
        analogInput = new AnalogInput(port);
        this.objectMinimumDistance = objectMinimumDistance;

    }

    /**
     * must be called periodically to update {@link #objectPresent}
     */
    public void run() {
        if (isInverted) {
            objectPresent = analogInput.getValue() <= objectMinimumDistance;
        } else {
            objectPresent = analogInput.getValue() >= objectMinimumDistance;
        }
    }

    /**
     * @return current state of the InfaredSensor, either true (detects object) or false (does not detect an object)
     */
    public boolean get() {
        return objectPresent;
    }

    /**
     * @return distance to compare with the analog value.
     */
    public int getObjectMinimumDistance() {
        return objectMinimumDistance;
    }

    /**
     * Sets a new minimumDistance
     *
     * @param objectMinimumDistance the distance to be set to evaluate with the analog value.
     */
    public void setObjectMinimumDistance(int objectMinimumDistance) {
        this.objectMinimumDistance = objectMinimumDistance;
    }

    /**
     * @return raw value from analog input.
     */
    public int getRaw() {
        return analogInput.getValue();
    }

    public boolean isInverted() {
        return isInverted;
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public void debug(String id) {
        SmartDashboard.putBoolean(id + "state", get());
        SmartDashboard.putNumber(id + "raw", getRaw());
    }
}
