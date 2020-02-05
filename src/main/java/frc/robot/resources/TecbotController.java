package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.ArrayList;
import java.util.List;

/**
 * A Tecbot Controller is a Joystick controller in which you can get different values from your controller,
 * whether it is a Ps4, Ps3, Xbox ONE, Xbox 360 or any other controller.
 */
public class TecbotController {

    private int currentPovAngle = -1;
    private int previousPovAngle = currentPovAngle;

    //POV Commands for 0°
    /**
     * Command for whenPressed state in POV 0
     */
    private CommandBase pov0CommandWhenPressed = null;

    /**
     * Command for whenReleased state in POV 0
     */
    private CommandBase pov0CommandWhenReleased = null;

    /**
     * Command for whileHeld state in POV 0
     */
    private CommandBase pov0CommandWhileHeld = null;

    //POV Commands for 90°
    /**
     * Command for whenPressed state in POV 90
     */
    private CommandBase pov90CommandWhenPressed = null;

    /**
     * Command for whenReleased state in POV 90
     */
    private CommandBase pov90CommandWhenReleased = null;

    /**
     * Command for whileHeld state in POV 90
     */
    private CommandBase pov90CommandWhileHeld = null;

    //POV Commands for 180°
    /**
     * Command for whenPressed state in POV 180
     */
    private CommandBase pov180CommandWhenPressed = null;

    /**
     * Command for whenReleased state in POV 180
     */
    private CommandBase pov180CommandWhenReleased = null;

    /**
     * Command for whileHeld state in POV 180
     */
    private CommandBase pov180CommandWhileHeld = null;

    //POV Commands for 270°
    /**
     * Command for whenPressed state in POV 270
     */
    private CommandBase pov270CommandWhenPressed = null;

    /**
     * Command for whenReleased state in POV 270
     */
    private CommandBase pov270CommandWhenReleased = null;

    /**
     * Command for whileHeld state in POV 270
     */
    private CommandBase pov270CommandWhileHeld = null;


    /**
     * The ports for the axis on the PS4 Controller in the following order:
     * <ul>
     *     <li>Left Axis X</li>
     *     <li>Left Axis Y</li>
     *     <li>Right Axis X</li>
     *     <li>Right Axis Y</li>
     * </ul>
     */
    private int[] portsJoysticksPS4 = {0, 1, 2, 5};

    /**
     * The ports for the axis on the XBOX Controller in the following order:
     * <ul>
     *     <li>Left Axis X</li>
     *     <li>Left Axis Y</li>
     *     <li>Right Axis X</li>
     *     <li>Right Axis Y</li>
     * </ul>
     */
    private int[] portsJoystickXBOX = {0, 1, 4, 5};

    /**
     * The ports for the buttons in PS4 controller.
     * <br>
     * In the following order:
     * <strong>
     * <ul>
     *     <li>a</li>
     *     <li>b</li>
     *     <li>x</li>
     *     <li>y</li>
     *     <li>lb</li>
     *     <li>rb</li>
     *     <li><i>BACK</i></li>
     *     <li><i>START</i></li>
     *     <li>LS</li>
     *     <li>RS</li>
     * </ul>
     * </strong>
     */
    private int[] portsButtonsPS4 = {2, 3, 1, 4, 5, 6, 9, 10, 11, 12};

    /**
     * The ports for the buttons in xbox controller.
     * <br>
     * In the following order:
     * <strong>
     * <ul>
     *     <li>a</li>
     *     <li>b</li>
     *     <li>x</li>
     *     <li>y</li>
     *     <li>lb</li>
     *     <li>rb</li>
     *     <li><i>BACK</i></li>
     *     <li><i>START</i></li>
     *     <li>LS</li>
     *     <li>RS</li>
     * </ul>
     * </strong>
     */
    private int[] portsButtonsXBOX = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /**
     * <h1>Trigger ports for PS4</h1>
     * <br>
     * In the following order:
     * <ul>
     *     <li>Left Trigger Value</li>
     *     <li>Right Trigger Value</li>
     * </ul>
     */
    private int[] portsTriggersPS4 = {3, 4};

    /**
     * <h1>Trigger ports for XBOX</h1>
     * <br>
     * In the following order:
     * <ul>
     *     <li>Left Trigger Value</li>
     *     <li>Right Trigger Value</li>
     * </ul>
     */
    private int[] portsTriggersXBOX = {2, 3};

    /**
     * Joystick object from FRC, this is used to getRawAxis,
     * etc.
     */
    private Joystick pilot;
    /**
     * {@link TypeOfController} enum object.
     */
    TypeOfController controllerType;
    /**
     * {@link #pilot} buttons.
     */
    List<JoystickButton> buttons;
    /**
     * Default offset to correct:<br>
     * {@link #getLeftAxisX()}<br>
     * {@link #getLeftAxisY()}<br>
     * {@link #getRightAxisX()}<br>
     * {@link #getRightAxisY()}<br>
     * {@link #getRawAxis(int, boolean)}
     *
     */
    private double offset = 0.1;

    /**
     * Controller Type that these (<br>
     * {@link #getLeftAxisX()}<br>
     * {@link #getLeftAxisY()}<br>
     * {@link #getRightAxisX()}<br>
     * {@link #getRightAxisY()}
     * ) methods support.
     */
    private enum TypeOfController {
        PS4,
        XBOX
    }

    /**
     * XBOX-style buttons.
     */
    public enum ButtonType {
        A,
        B,
        X,
        Y,
        LB,
        RB,
        BACK,
        START,
        LS,
        RS,
        POV_0,
        POV_90,
        POV_180,
        POV_270
    }

    /**
     * @param port The port that the controller has in the Driver Station.
     */
    public TecbotController(int port) {
        pilot = new Joystick(port);

        controllerType = null;
        if (pilot.getName().toLowerCase().contains("wireless controller")) controllerType = TypeOfController.PS4;
        if (pilot.getName().toLowerCase().contains("xbox")) controllerType = TypeOfController.XBOX;

        if (pilot.getName() == null) DriverStation.reportWarning("Joystick not found (Tecbot Controller)", true);
        if (controllerType != null) setButtons();
        else DriverStation.reportWarning("Controller not identified, some methods will return 0.", false);

    }

    /**
     * This function will return the value of the Left Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getLeftAxisX() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[0]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[0]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getLeftAxisX(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * This function will return the value of the Left Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getLeftAxisY() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[1]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[1]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getLeftAxisY(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }


    /**
     * This function will return the value of the Right Axis <i>X</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getRightAxisX() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[2]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[2]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisX(). Returned 0. Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, offset);
    }

    /**
     * This function will return the value of the Right Axis <i>Y</i>.
     * <br>Ranges from -1 to 1.
     *
     * @return axis value
     */
    public double getRightAxisY() {
        double value;
        switch (controllerType) {
            case PS4:
                value = pilot.getRawAxis(portsJoysticksPS4[3]);
                break;
            case XBOX:
                value = pilot.getRawAxis(portsJoystickXBOX[3]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getRightAxisY(). Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, getOffset());
    }

    /**
     * Returns value of given axis.
     *
     * @param axis axis port in the controller.
     * @return value of given axis.
     */
    public double getRawAxis(int axis, boolean ground) {
        return ground ? ground(pilot.getRawAxis(axis), offset) : pilot.getRawAxis(axis);
    }

    /**
     * Returns value of given axis.
     *
     * @param axis axis port in the controller.
     * @return value of given axis.
     */
    public double getRawAxis(int axis) {
        return pilot.getRawAxis(axis);
    }

    /**
     * Returns the value of the button.
     *
     * @param buttonNumber The button to be read.
     * @return The state of the button.
     */
    public boolean getRawButton(int buttonNumber) {
        return pilot.getRawButton(buttonNumber);
    }

    /**
     * @return Returns triggers in controller.
     * <br>When the triggers are idle, 0 will be returned.
     * <br>When the right trigger is pressed, it will return a positive
     * value.
     * <br>When the left trigger is pressed, it will return a negative value.
     * <br>Therefore, both triggers pressed will return 0.
     */
    public double getTriggers() {
        double value;
        switch (controllerType) {
            case PS4:
                value = (pilot.getRawAxis(portsTriggersPS4[1]) - pilot.getRawAxis(portsTriggersPS4[0])) / 2;
                break;
            case XBOX:
                value = pilot.getRawAxis(portsTriggersXBOX[1]) - pilot.getRawAxis(portsTriggersXBOX[0]);
                break;
            default:
                value = 0;
                DriverStation.reportWarning("Could not get axis value from getTriggers(). Returned 0. Use getAxisValue() instead.", false);
                break;
        }
        return ground(value, offset);
    }

    private void setButtons() {
        List<JoystickButton> bs = new ArrayList<>();
        switch (controllerType) {
            case XBOX:
                for (int port : portsButtonsXBOX) {
                    bs.add(new JoystickButton(pilot, port));
                }
                break;
            case PS4:
                for (int port : portsButtonsPS4) {
                    bs.add(new JoystickButton(pilot, port));
                }
                break;
            default:

                for (int i = 0; i < pilot.getButtonCount(); i++) {
                    bs.add(new JoystickButton(pilot, i + 1));
                }
                break;
        }
        buttons = bs;
    }

    /**
     * @param button the button to return
     * @return Returns JoystickButton Object
     */
    public JoystickButton getButton(ButtonType button) {
        int index = 0;
        switch (button) {
            case A:
                //nothing needs to be done here because index already = 0
                break;
            case B:
                index = 1;
                break;
            case X:
                index = 2;
                break;
            case Y:
                index = 3;
                break;
            case LB:
                index = 4;
                break;
            case RB:
                index = 5;
                break;
            case BACK:
                index = 6;
                break;
            case START:
                index = 7;
                break;
            case LS:
                index = 8;
                break;
            case RS:
                index = 9;
                break;
            default:
                DriverStation.reportError("Button not recognized. TecbotController", true);
                break;
        }
        return buttons.get(index);

    }

    public void whenPressed(ButtonType buttonType, CommandBase command) {
        switch (buttonType) {
            case POV_0:
                pov0CommandWhenPressed = command;
                break;
            case POV_90:
                pov90CommandWhenPressed = command;
                break;
            case POV_180:
                pov180CommandWhenPressed = command;
                break;
            case POV_270:
                pov270CommandWhenPressed = command;
                break;
            default:
                JoystickButton m_button = getButton(buttonType);
                m_button.whenPressed(command);
                break;
        }
    }

    public void whenReleased(ButtonType buttonType, CommandBase command) {
        switch (buttonType) {
            case POV_0:
                pov0CommandWhenReleased = command;
                break;
            case POV_90:
                pov90CommandWhenReleased = command;
                break;
            case POV_180:
                pov180CommandWhenReleased = command;
                break;
            case POV_270:
                pov270CommandWhenReleased = command;
                break;
            default:
                JoystickButton m_button = getButton(buttonType);
                m_button.whenReleased(command);
        }
    }

    /**
     * Constantly starts the given command while the button is held.
     * <p>
     * {@link Command#schedule(boolean)} will be called repeatedly while the button is held, and will
     * be canceled when the button is released.  The command is set to be interruptible.
     *
     * <strong>The command should always have the isFinished value as true, since it
     * will be scheduled several times.</strong>
     *
     * @param command    the command to start
     * @param buttonType the button type to refer to
     * @return this button, so calls can be chained
     */
    public void whileHeld(ButtonType buttonType, CommandBase command) {
        switch (buttonType) {
            case POV_0:
                pov0CommandWhileHeld = command;
                break;
            case POV_90:
                pov90CommandWhileHeld = command;
                break;
            case POV_180:
                pov180CommandWhileHeld = command;
                break;
            case POV_270:
                pov270CommandWhileHeld = command;
                break;
            default:
                JoystickButton m_button = getButton(buttonType);
                m_button.whileHeld(command);
        }
    }

    /**
     * Must be called in teleop Periodic to set POV data.
     */
    public void run() {
        //sets currentPovAngle to POV angle from pilot
        currentPovAngle = pilot.getPOV();
        //sets current commands based on currentPovAngle
        CommandBase currentPovWhenPressedCommand = getPovWhileHeldCommand(currentPovAngle);
        //CommandBase currentPovWhenReleasedCommand = getPovWhileHeldCommand(currentPovAngle);
        CommandBase currentPovWhileHeldCommand = getPovWhileHeldCommand(currentPovAngle);

        //sets previous commands based on previousPovAngle
        CommandBase previousPovWhenPressedCommand = getPovWhileHeldCommand(previousPovAngle);
        CommandBase previousPovWhenReleasedCommand = getPovWhileHeldCommand(previousPovAngle);
        CommandBase previousPovWhileHeldCommand = getPovWhileHeldCommand(previousPovAngle);


        /*
        if there is a button change (e.g. currentPovAngle differs from previousPovAngle),
        then the previous whileHeld command will be cancelled,
        the previous whenReleased command will be scheduled,
        and the current whenPressed command will be scheduled.
        */
        if (currentPovAngle != previousPovAngle) {
            if (previousPovWhenReleasedCommand != null)
                previousPovWhenReleasedCommand.schedule();
            if (previousPovWhileHeldCommand != null)
                previousPovWhileHeldCommand.cancel();
            if (currentPovWhenPressedCommand != null)
                currentPovWhenPressedCommand.schedule();

            /*
            clears all previous commands, this a requirement
            if from code a group command is manually scheduled
            and is going to be scheduled again.
            This is not necessary for 'regular' buttons, since
            that is automatically done by frc / wpilib libraries
            */
            clearGroupedCommands(
                    previousPovWhenPressedCommand,
                    previousPovWhenReleasedCommand,
                    previousPovWhileHeldCommand
            );
        }
        //this will just schedule the whileHeld command,
        //which should have the isFinised true.
        //this command also has to be cleared since it will be called several times.
        if (currentPovWhileHeldCommand != null) {
            currentPovWhileHeldCommand.schedule();
            clearGroupedCommands(currentPovWhileHeldCommand);
        }


        previousPovAngle = currentPovAngle;
    }

    /**
     * Returns new {@link CommandBase} given the POV angle.
     *
     * @param angle POV angle
     * @return {@link CommandBase} for angle in whenPressed state.
     */
    public CommandBase getPovWhenPressedCommand(int angle) {
        switch (angle) {
            case 0:
                return pov0CommandWhenPressed;
            case 90:
                return pov90CommandWhenPressed;
            case 180:
                return pov180CommandWhenPressed;
            case 270:
                return pov270CommandWhenPressed;
            default:
                return null;
        }
    }

    /**
     * Returns new {@link CommandBase} given the POV angle.
     *
     * @param angle POV angle
     * @return {@link CommandBase} for angle in whenReleased state.
     */
    public CommandBase getPovWhenReleasedCommand(int angle) {
        switch (angle) {
            case 0:
                return pov0CommandWhenReleased;
            case 90:
                return pov90CommandWhenReleased;
            case 180:
                return pov180CommandWhenReleased;
            case 270:
                return pov270CommandWhenReleased;
            default:
                return null;
        }
    }

    /**
     * Returns new {@link CommandBase} given the POV angle.
     *
     * @param angle POV angle
     * @return {@link CommandBase} for angle in whileHeld state.
     */
    public CommandBase getPovWhileHeldCommand(int angle) {
        switch (angle) {
            case 0:
                return pov0CommandWhileHeld;
            case 90:
                return pov90CommandWhileHeld;
            case 180:
                return pov180CommandWhileHeld;
            case 270:
                return pov270CommandWhileHeld;
            default:
                return null;
        }
    }

    /**
     * Set the rumble output for the HID. The DS currently supports 2 rumble values, left rumble and
     * right rumble.
     *
     * @param rumbleType Which rumble value to set
     * @param value      The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(GenericHID.RumbleType rumbleType, double value) {
        pilot.setRumble(rumbleType, value);
    }

    /**
     * By default, offset equals 0.1, meaning that if any of the axis is
     * equal to or less than 0.1 and greater than or equal to -0.1, it will
     * return 0.
     *
     * @param offset The offset that it will have on all axis.
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * @return the amount of offset that it will correct.
     */
    public double getOffset() {
        return this.offset;
    }

    /**
     * This function will correct a given value according to the given offset.
     *
     * @param value  raw value.
     * @param offset positive double less than the value.
     * @return corrected value.
     */
    private double ground(double value, double offset) {
        return value >= -offset && value <= offset ? 0 : value;
    }

    /**
     * Clears any amount of group commands, this means that the same
     * instance of the group command can be reused and run again.
     *
     * @param commandGroupBases {@link CommandGroupBase}
     */
    private void clearGroupedCommands(CommandBase... commandGroupBases) {
        for (CommandBase commandGroup :
                commandGroupBases) {
            CommandGroupBase.clearGroupedCommand(commandGroup);
        }
    }

    private boolean notNullCommands(CommandBase... commandBases) {
        if (commandBases.length < 1) return false;
        boolean notNull = false;
        for (CommandBase command : commandBases) {
            if (commandBases == null) return false;
            else notNull = true;
        }
        return notNull;
    }

}