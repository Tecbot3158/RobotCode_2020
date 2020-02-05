package frc.robot.resources;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.robotStates.TestGroupParallelCommand;

/**
 * This enum contains a set of every {@link SequentialCommandGroup} which could
 * be a possible usable state for the robot.
 */
public enum RobotCurrentStateCommand {
    ALL_SYSTEMS_OFF(RobotStateCommands.GROUP_COMMAND_ALL_SYSTEMS_OFF, "ASO"),
    FRONT_INTAKE_AND_TRANSPORT_ONLY(RobotStateCommands.GROUP_COMMAND_FRONT_INTAKE_AND_TRANSPORT_ONLY, "FI_TO_NS"),
    REAR_INTAKE_AND_TRANSPORT_ONLY(RobotStateCommands.GROUP_COMMAND_REAR_INTAKE_AND_TRANSPORT_ONLY, "RI_TO_NS");

    public CommandGroupBase command;

    String id;

    int schedulesCount = 0;
    int cancellationCount = 0;

    static int globalSchedulesCount = 0;
    static int globalCancellationCount = 0;

    RobotCurrentStateCommand(CommandGroupBase commandGroup, String id) {
        this.command = commandGroup;
        this.id = id;
    }

    /**
     * @return {@link ParallelCommandGroup} command from its enum element.
     */
    public CommandGroupBase getCommand() {
        return this.command;
    }

    /**
     * Adds 1 to {@link #schedulesCount} and {@link #globalSchedulesCount}
     */
    void addToScheduleVariable() {
        this.getCommand().schedule();
        this.schedulesCount++;
        globalSchedulesCount++;
    }

    /**
     * Adds 1 to {@link #cancellationCount} and {@link #globalCancellationCount}
     */
    void addToCancelVariable() {
        // this.getCommand().cancel();
        this.cancellationCount++;
        globalCancellationCount++;
    }

    /**
     * @param self   If true, sends everything about that command: run state (e.g.
     *               true if running, false if finished), schedules count (the
     *               amount of times that {@link #addToScheduleVariable()} has been
     *               called, cancellation count (the amount of times that
     *               {@link #addToCancelVariable()} has been called.
     * @param global If true, sends everything in global terms: global schedule
     *               count and cancellation count.
     */
    void sendCommandData(boolean self, boolean global) {
        if (global) {
            SmartDashboard.putNumber("GSCHEDULE", globalSchedulesCount);
            SmartDashboard.putNumber("GCANCEL", globalCancellationCount);
        }
        if (self) {
            SmartDashboard.putBoolean(this.toString() + "RUN", !this.command.isFinished());
            SmartDashboard.putNumber(this.id + "-SCH", schedulesCount);
            SmartDashboard.putNumber(this.id + "-CLS", cancellationCount);
        }
    }
}

class RobotStateCommands {

    /**
     * <h3><strong>ALL SYSTEMS OFF</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Off mode, deflector off</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #OFF</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_ALL_SYSTEMS_OFF = new TestGroupParallelCommand();
    /**
     * <h3><strong>TRANSPORT active when Power Cell present in Front
     * INTAKE</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on intake mode, pneumatics on</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector on</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>off</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_FRONT_INTAKE_AND_TRANSPORT_ONLY = new TestGroupParallelCommand();


    /**
     * <h3><strong>TRANSPORT active when Power Cell present in Rear
     * INTAKE</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on intake mode, pneumatics on</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector on</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #OFF</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_REAR_INTAKE_AND_TRANSPORT_ONLY = new TestGroupParallelCommand();

    /**
     * <h3><strong>INTAKE GOTO BOTTOM PORT</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on shooter mode, pneumatics off</li>
     * <li>Rear intake on intake mode, pneumatics on</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector on</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #OFF</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_INTAKE_GOTO_BOTTOM_PORT = new TestGroupParallelCommand();

    /**
     * <h3><strong>INTAKE FEEDER AND TRANSPORT</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on intake mode, pneumatics off</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector ON</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #OFF</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_INTAKE_FEEDER_AND_TRANSPORT = new TestGroupParallelCommand();

    /**
     * <h3><strong>SHOOT AND TRANSPORT</strong></h3>
     * <ul>
     *
     * <li>Intakes:
     * <ul>
     * <li>Front intake on off mode, pneumatics off</li>
     * <li>Rear intake on off mode, pneumatics off</li>
     * </ul>
     * </li>
     *
     * <li>Power Cell Transportation System
     * <ul>
     * <li>Forward mode, deflector off</li>
     * </ul>
     * </li>
     *
     * <li>Powercell shooter:
     * <ul>
     * <li>On position #</li>
     * </ul>
     * </li>
     *
     * </ul>
     */
    public static ParallelCommandGroup GROUP_COMMAND_SHOOT_AND_TRANSPORT = new TestGroupParallelCommand();

}