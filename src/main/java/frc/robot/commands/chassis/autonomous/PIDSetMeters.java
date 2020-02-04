/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDSetMeters extends InstantCommand {

    double m_angle, deltaCount;
    double CONFIG_NOT_SET = -58005456;

    public PIDSetMeters(double meters) {
        deltaCount = meters * TecbotConstants.K_CHASSIS_METERS_TO_ENCODER;
        m_angle = CONFIG_NOT_SET;
    }
    public PIDSetMeters(double meters, double angle) {
        deltaCount = meters * TecbotConstants.K_CHASSIS_METERS_TO_ENCODER;
        m_angle = angle;
    }

    @Override
    public void initialize() {
        Robot.m_robotContainer.getDriveTrain().setPidStraightTarget(TecbotSensors.getEncoderRaw(TecbotSensors.SubsystemType.LEFT_CHASSIS) + deltaCount);

        if(m_angle != CONFIG_NOT_SET){
            Robot.m_robotContainer.getDriveTrain().setPidAngleTarget( m_angle);
        }else {
            Robot.m_robotContainer.getDriveTrain().setPidAngleTarget(TecbotSensors.getYaw());
        }

    }
}
