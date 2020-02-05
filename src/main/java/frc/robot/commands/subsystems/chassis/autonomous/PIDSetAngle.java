/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.chassis.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;


public class PIDSetAngle extends InstantCommand {
    double m_angle;
    boolean m_isDelta;

    public PIDSetAngle(double angle) {
        m_angle = angle;
    }

    /**
     * @param isDelta true if the angle indicated is the amount of degrees to turn instead of the desired heading
     */

    public PIDSetAngle(double angle, boolean isDelta) {
        m_angle = angle;
        m_isDelta = isDelta;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().setPidAngleTarget(m_isDelta ? Robot.getRobotContainer().getTecbotSensors().getYaw() : 0 + m_angle);
    }
}
