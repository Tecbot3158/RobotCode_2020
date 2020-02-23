package frc.robot.resources;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.Intake;

public class TecbotSensors {

    private Navx tecbotGyro;

    private TecbotEncoder leftChassisEncoder, rightChassisEncoder, middleChassisEncoder,
            sharedMotorsRightEncoder, sharedMotorsLeftEncoder;

    //Color sensor
    private ColorSensorV3 colorSensorV3;
    private ColorMatch colorMatcher;
    //Color sensor constants
    private final I2C.Port I2C_PORT_ONBOARD = I2C.Port.kOnboard;
    private final Color K_BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color K_GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color K_RED_TARGET = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color K_YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.524, 0.113);

    //CLIMBER stuff
    private DigitalInput climberLimitSwitch;


    //power cell count
    private DigitalInput infraredIntakeSensor, infraredShooterSensor;
    private int currentPowerCellCount;
    private boolean currentStateIntakeIRSensor, currentStateShooterIRSensor,
            previousStateIntakeIRSensor, previousStateShooterIRSensor;

    public TecbotSensors() {

    }

    /**
     * Initializes all sensors.
     */
    public void initializeAllSensors() {

        tecbotGyro = new Navx();
        sharedMotorsRightEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderRight(),
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[1]);
        sharedMotorsLeftEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderLeft(),
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[1]);

        leftChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS[1]);
        rightChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS[1]);
        middleChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_MIDDLE_CHASSIS_MOTOR_WITH_ENCODER)
                        , RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS[1]);
        if (RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_IS_INVERTED && leftChassisEncoder != null)
            leftChassisEncoder.setInverted(true);
        if (RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_IS_INVERTED && rightChassisEncoder != null)
            rightChassisEncoder.setInverted(true);
        if (RobotMap.DRIVE_TRAIN_MIDDLE_CHASSIS_ENCODER_IS_INVERTED && middleChassisEncoder != null)
            middleChassisEncoder.setInverted(true);

        colorSensorV3 = new ColorSensorV3(I2C_PORT_ONBOARD);
        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(K_BLUE_TARGET);
        colorMatcher.addColorMatch(K_GREEN_TARGET);
        colorMatcher.addColorMatch(K_RED_TARGET);
        colorMatcher.addColorMatch(K_YELLOW_TARGET);

        climberLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LIMIT_SWITCH_PORT);

        infraredIntakeSensor = new DigitalInput(RobotMap.TRANSPORTATION_SYSTEM_INFRARED_INTAKE_SENSOR_PORT);
        infraredShooterSensor = new DigitalInput(RobotMap.TRANSPORTATION_SYSTEM_INFRARED_SHOOTER_SENSOR_PORT);
        setCurrentPowerCellCount(0);
        currentStateIntakeIRSensor = false;
        currentStateShooterIRSensor = false;
        previousStateIntakeIRSensor = false;
        previousStateShooterIRSensor = false;
    }

    /**
     * Must be called to update tecbotGyro data and InfraRed sensors for
     * Power cell counting.
     */
    public void sensorsPeriodic() {
        tecbotGyro.run();
        currentStateIntakeIRSensor = powerCellPresentInIntake();
        currentStateShooterIRSensor = powerCellPresentInShooter();

        //change detected in intake (detected power cell and now is not present)
        //if condition true, a power cell has gotten inside the transportation system.
        if (previousStateIntakeIRSensor && !currentStateIntakeIRSensor)
            addToPowerCellCount();

        //change detected in shooter (detected power cell and now is not present)
        //if condition true, a power cell has left the shooter
        if (previousStateShooterIRSensor && !currentStateShooterIRSensor)
            subtractFromPowerCellCount();

        previousStateIntakeIRSensor = currentStateIntakeIRSensor;
        previousStateShooterIRSensor = currentStateShooterIRSensor;
    }

    /**
     * @return {@link Navx} object of tecbotGyro
     */
    public Navx getTecbotGyro() {
        return tecbotGyro;
    }

    /**
     * @return angle between -180 and 180.
     */
    public double getYaw() {
        return tecbotGyro.getYaw();
    }

    /**
     * @return color sensor's current color as a {@link Intake.Color} enum.
     */
    public Intake.Color getColor() {
        final ColorMatchResult match = colorMatcher.matchClosestColor(colorSensorV3.getColor());

        if (match.color == K_BLUE_TARGET) {
            return Intake.Color.BLUE;
        } else if (match.color == K_RED_TARGET) {
            return Intake.Color.RED;
        } else if (match.color == K_GREEN_TARGET) {
            return Intake.Color.GREEN;
        } else if (match.color == K_YELLOW_TARGET) {
            return Intake.Color.YELLOW;
        } else {
            return null;
        }
    }

    /**
     * @return Raw value from selected encoder
     */
    public double getEncoderRaw(SubsystemType subsystem) {
        switch (subsystem) {
            case RIGHT_CHASSIS:
                return rightChassisEncoder.getRaw();
            case LEFT_CHASSIS:
                return leftChassisEncoder.getRaw();
            case MIDDLE_CHASSIS:
                return middleChassisEncoder.getRaw();
            case SHOOTER:
                return RobotMap.SHOOTER_ENCODER_IN_RIGHT_MOTOR ?
                        sharedMotorsRightEncoder.getRaw() :
                        sharedMotorsLeftEncoder.getRaw();
            default:
                return 0;
        }
    }

    public TecbotEncoder getEncoder(SubsystemType subsystem){
        switch (subsystem) {
            case RIGHT_CHASSIS:
                return rightChassisEncoder;
            case LEFT_CHASSIS:
                return leftChassisEncoder;
            case MIDDLE_CHASSIS:
                return middleChassisEncoder;
            case SHOOTER:
                return RobotMap.SHOOTER_ENCODER_IN_RIGHT_MOTOR ?
                        sharedMotorsRightEncoder :
                        sharedMotorsLeftEncoder;
            default:
                return null;
        }
    }

    /**
     * subsystem type
     */
    public enum SubsystemType {
        MIDDLE_CHASSIS, RIGHT_CHASSIS, LEFT_CHASSIS, SHOOTER
    }

    /**
     * @return raw limit switch value.
     */
    public boolean getClimberLimitSwitch() {
        return climberLimitSwitch.get();
    }

    /**
     * @return current powerCellCount
     */
    public int getCurrentPowerCellCount() {
        return this.currentPowerCellCount;
    }

    /**
     * sets {@link #currentPowerCellCount}
     *
     * @param value value to set to powerCellCount
     */
    private void setCurrentPowerCellCount(int value) {
        this.currentPowerCellCount = value;
    }

    public boolean powerCellPresentInIntake() {
        return this.infraredIntakeSensor.get();
    }

    public boolean powerCellPresentInShooter() {
        return this.infraredShooterSensor.get();
    }

    /**
     * Adds 1 unit (POWER CELL) from {@link #currentPowerCellCount}
     */
    public void addToPowerCellCount() {
        addToPowerCellCount(1);
    }

    /**
     * Subtracts from {@link #currentPowerCellCount}
     *
     * @param value the value to add to {@link #currentPowerCellCount}
     */
    public void addToPowerCellCount(int value) {
        this.setCurrentPowerCellCount((int) (getCurrentPowerCellCount() + Math.abs(value)));
    }

    /**
     * Subtracts 1 unit (POWER CELL) from {@link #currentPowerCellCount}
     */
    public void subtractFromPowerCellCount() {
        subtractFromPowerCellCount(1);
    }

    /**
     * Subtracts from {@link #currentPowerCellCount}
     *
     * @param value positive value to subtract from the {@link #currentPowerCellCount}
     */
    public void subtractFromPowerCellCount(int value) {
        this.setCurrentPowerCellCount((int) (getCurrentPowerCellCount() - Math.abs(value)));
    }
}
