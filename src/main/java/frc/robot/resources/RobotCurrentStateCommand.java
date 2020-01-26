package frc.robot.resources;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;

/**
 * This enum contains a set of every {@link SequentialCommandGroup} which could be a possible
 * usable state for the robot.
 */
public enum RobotCurrentStateCommand {
    ALL_SYSTEMS_OFF(RobotMap.GROUP_COMMAND_ALL_SYSTEMS_OFF),
    FRONT_INTAKE_AND_TRANSPORT_ONLY(RobotMap.GROUP_COMMAND_FRONT_INTAKE_AND_TRANSPORT_ONLY),
    REAR_INTAKE_AND_TRANSPORT_ONLY(RobotMap.GROUP_COMMAND_FRONT_INTAKE_AND_TRANSPORT_ONLY);


    public SequentialCommandGroup command;
    static int globalCommandsCountCalls = 0;
    int commandCountCalls = 0;

    RobotCurrentStateCommand(SequentialCommandGroup commandGroup) {
        this.command = commandGroup;
    }

    /**
     * @return {@link SequentialCommandGroup} command from its enum element.
     */
    SequentialCommandGroup getCommand() {
        globalCommandsCountCalls++;
        commandCountCalls++;
        return this.command;
    }

    /**
     * this will send the following data to the <i>SmartDashboard</i>
     * <ul>
     *     <li>{COMMAND_NAME} + RUN - <strong>boolean</strong> - true if commandStillRunning</li>
     *     <li>{COMMAND_NAME} + count - <strong>integer</strong> - amount of times this command has been called to do something.
     *     Either cancelling it or starting it.</li>
     *     <li>{COMMAND_NAME} + gCount - <strong>integer</strong> - amount of times <strong>ALL</strong> commands
     *     in the {@link RobotCurrentStateCommand} enum has been called. Either cancelling or starting it.</li>
     * </ul>
     */
    void setCommandData() {
        SmartDashboard.putBoolean(this.toString() + "RUN", !this.command.isFinished());
        SmartDashboard.putNumber(this.toString() + "COUNT", this.commandCountCalls);
        SmartDashboard.putNumber(this.toString() + "GCOUNT", globalCommandsCountCalls);
    }

    /**
     * this will send the following data to the <i>{@link SmartDashboard}</i>. <strong>With this method
     * you can customize which data is sent by setting the three parameters to true or false.</strong>
     * <ul>
     *     <li>{COMMAND_NAME} + RUN - <strong>boolean</strong> - true if commandStillRunning</li>
     *     <li>{COMMAND_NAME} + count - <strong>integer</strong> - amount of times this command has been called to do something.
     *     Either cancelling it or starting it.</li>
     *     <li>{COMMAND_NAME} + gCount - <strong>integer</strong> - amount of times <strong>ALL</strong> commands
     *     in the {@link RobotCurrentStateCommand} enum has been called. Either cancelling or starting it.</li>
     * </ul>
     *
     * @param run   If true, a boolean will be sent to the {@link SmartDashboard} to inform whether the
     *              command is still running or not.
     * @param count If true, an integer will be sent to the {@link SmartDashboard}, informing of the
     *              amount of times this command has been called.
     * @param gCount If true, an integer will be sent to the {@link SmartDashboard} to inform the
     *               amount of times <strong>ALL commands have been called.</strong>
     */
    void setCommandData(boolean run, boolean count, boolean gCount) {
        SmartDashboard.putBoolean(this.toString() + "RUN", !this.command.isFinished());
        SmartDashboard.putNumber(this.toString() + "COUNT", this.commandCountCalls);
        SmartDashboard.putNumber(this.toString() + "GCOUNT", globalCommandsCountCalls);
    }
}
