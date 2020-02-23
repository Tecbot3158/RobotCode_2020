/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.powerCellCounter;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.Math;

public class PowerCellCounter extends SubsystemBase {
    /**
     * Creates a new Subsystem.
     */

    private DigitalOutput red, green, blue;
    private boolean previousStateFrontIntakeIRSensor = false,
            previousStateRearIntakeIRSensor = false,
            previousStateFrontRearIRSensor = false;

    private boolean currentStateFrontIntakeIRSensor = false,
            currentStateRearIntakeIRSensor = false,
            currentStateFrontRearIRSensor = false;

    private int powerCellCount = 0;


    public PowerCellCounter() {
        red = new DigitalOutput(RobotMap.POWER_CELL_COUNTER_RED_DIGITAL_OUTPUT_PORT);
        green = new DigitalOutput(RobotMap.POWER_CELL_COUNTER_GREEN_DIGITAL_OUTPUT_PORT);
        blue = new DigitalOutput(RobotMap.POWER_CELL_COUNTER_BLUE_DIGITAL_OUTPUT_PORT);

    }

    public void setColorOff() {
        setRed(false);
        setGreen(false);
        setBlue(false);
    }

    public void setRed(boolean value) {
        red.set(value);
    }

    public void setGreen(boolean value) {
        green.set(value);
    }

    public void setBlue(boolean value) {
        blue.set(value);
    }

    public void setColor(boolean red, boolean green, boolean blue) {
        setRed(red);
        setGreen(green);
        setBlue(blue);
    }

    public void addToPowerCellCount() {
        powerCellCount++;
        powerCellCount = (int) Math.clamp(powerCellCount, 0, 5);
    }

    public void subtractFromPowerCellCount() {
        powerCellCount--;
        powerCellCount = (int) Math.clamp(powerCellCount, 0, 5);
    }

    public int getPowerCellCount() {
        return powerCellCount;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}