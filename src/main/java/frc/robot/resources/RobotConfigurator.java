/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

/**
 * This class is used for building objects such as:<br>
 * {@link TecbotEncoder}<br>
 * {@link TecbotMotorList}<br>
 * {@link DoubleSolenoid}<br>
 * {@link Solenoid}<br>
 * {@link Servo}<br>
 */
public class RobotConfigurator {

    /**
     * This constant is intended to be used when creating a new TecbotEncoder
     * which is connected to a TALON_SRX, and in that case this variable replaces port <i>a</i>
     * and <i>b</i>
     */
    public static final int CONFIG_NOT_SET = -1;

    /**
     * This method is use to create a new TecbotEncoder, it can be either two-channel or just linked
     * to a TALON_SRX speed controller.
     *
     * @param speedController the speed controller that the encoder is linked to.
     * @param a               If encoder is a two-channel sensor, this should correspond to the first channel,
     *                        otherwise use {@link #CONFIG_NOT_SET}
     * @param b               If encoder is a two-channel sensor, this should correspond to the second channel,
     *                        otherwise use {@link #CONFIG_NOT_SET}
     * @return {@link TecbotEncoder}
     */
    public static TecbotEncoder buildEncoder(TecbotSpeedController speedController, int a, int b) {
        if (speedController != null) {

            if (speedController.getType() == TypeOfMotor.TALON_SRX)
                return new TecbotEncoder(speedController);

        }
        if (a != CONFIG_NOT_SET && b > CONFIG_NOT_SET && a != b)
            return new TecbotEncoder(a, b);

        DriverStation.reportWarning("No ENCODER on this build try", true);
        return null;
    }

    /**
     * This method is intended to be used when creating a new instance of {@link TecbotMotorList},
     * and will receive 3 parameters:
     * <br> ports, invertedMotors, and motorTypes
     *
     * @param ports              ports in which the speed controllers are located.
     * @param invertedMotorPorts the ports of the motors that are inverted.
     * @param motorTypes         array of {@link TecbotSpeedController.TypeOfMotor}
     * @return {@link TecbotMotorList} with motor list.
     */
    public static TecbotMotorList buildMotorList(int[] ports, int[] invertedMotorPorts, TypeOfMotor[] motorTypes) {
        return new TecbotMotorList(ports, invertedMotorPorts, motorTypes);
    }

    /**
     * Intended to be used when creating a new instance of {@link DoubleSolenoid}
     *
     * @param ports array with solenoid ports.
     * @return {@link DoubleSolenoid} new instance
     */
    public static DoubleSolenoid buildDoubleSolenoid(int[] ports) {
        return new DoubleSolenoid(ports[0], ports[1]);
    }

    /**
     * Intended to be used when creating a new instance of {@link Solenoid}
     *
     * @param ports array with solenoid ports.
     * @return {@link Solenoid} new instance
     */
    public static Solenoid buildSingleSolenoid(int[] ports) {
        return new Solenoid(ports[0], ports[1]);
    }

    /**
     * Builds new {@link Servo} object.
     * @param port servo port
     * @return {@link Servo}
     */
    public static Servo buildServo(int port) {
        return new Servo(port);
    }

}