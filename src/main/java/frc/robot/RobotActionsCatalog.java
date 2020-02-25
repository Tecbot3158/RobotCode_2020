package frc.robot;

import frc.robot.commands.robotActions.AllSystemsOff;
import frc.robot.commands.robotActions.TransportDeflectorOff;
import frc.robot.commands.robotActions.intakeTransport.*;
import frc.robot.commands.robotActions.mixed.*;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromInitiationLineCompensate;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromTargetZoneCompensate;
import frc.robot.commands.robotActions.shootCompensateAndTransport.ShootFromTrenchCompensate;
import frc.robot.commands.robotActions.shootTransport.NoPIDShootTrenchAndTransport;
import frc.robot.commands.robotActions.shootTransport.ShootInitiationLineAndTransport;
import frc.robot.commands.robotActions.shootTransport.ShootTargetZoneAndTransport;
import frc.robot.commands.robotActions.shootTransport.ShootTrenchAndTransport;
import frc.robot.commands.subsystemCommands.pctower.TransportationSystemShootingSpeed;
import frc.robot.commands.subsystemCommands.shooter.ShootFromInitiationLine;
import frc.robot.commands.subsystemCommands.shooter.ShootFromTargetZone;
import frc.robot.commands.subsystemCommands.shooter.ShootFromTrench;
import frc.robot.commands.subsystemCommands.shooter.ShooterOff;

public class RobotActionsCatalog {
    private static RobotActionsCatalog instance;



    private TransportationSystemShootingSpeed transportationSystemShootingSpeed;

    private FrontOutTakeAndTransport frontOutTakeAndTransport;

    private AllSystemsOff allSystemsOff;

    private FrontIntakeAndTransport frontIntakeAndTransport;
    private IntakeFromFeederAndTransport intakeFromFeederAndTransport;
    private RearIntakeAndShootBottomPort rearIntakeAndShootBottomPort;
    private RearIntakeAndTransport rearIntakeAndTransport;

    private RearIntakeShootTrenchTransport rearIntakeShootTrenchTransport;
    private RearIntakeShootTargetZoneTransport rearIntakeShootTargetZoneTransport;
    private RearIntakeShootInititationLineTransport rearIntakeShootInitiationLineTransport;


    private FrontIntakeShootTrenchTransport frontIntakeShootTrenchTransport;
    private FrontIntakeShootTargetZoneTransport frontIntakeShootTargetZoneTransport;
    private FrontIntakeShootInitiationLineTransport frontIntakeShootInitiationLineTransport;

    private IntakesAndTransportOff intakesAndTransportOff;

    private ShootInitiationLineAndTransport shootInitiationLineAndTransport;
    private ShootTargetZoneAndTransport shootTargetZoneAndTransport;
    private ShootTrenchAndTransport shootTrenchAndTransport;

    private NoPIDShootTrenchAndTransport noPIDShootTrenchAndTransport;

    private ShootFromTrench shootFromTrench;
    private ShootFromInitiationLine shootFromInitiationLine;
    private ShootFromTargetZone shootFromTargetZone;
    private ShooterOff shooterOff;

    private TransportDeflectorOff transportDeflectorOff;

    private ShootFromTrenchCompensate shootFromTrenchCompensate;
    private ShootFromInitiationLineCompensate shootFromInitiationLineCompensate;
    private ShootFromTargetZoneCompensate shootFromTargetZoneCompensate;


    public RobotActionsCatalog() {

        transportationSystemShootingSpeed = new TransportationSystemShootingSpeed();

        frontOutTakeAndTransport = new FrontOutTakeAndTransport();

        allSystemsOff = new AllSystemsOff();

        frontIntakeAndTransport = new FrontIntakeAndTransport();
        intakeFromFeederAndTransport = new IntakeFromFeederAndTransport();
        rearIntakeAndShootBottomPort = new RearIntakeAndShootBottomPort();
        rearIntakeAndTransport = new RearIntakeAndTransport();

        //rear intake shoot
        rearIntakeShootTrenchTransport = new RearIntakeShootTrenchTransport();
        rearIntakeShootTargetZoneTransport = new RearIntakeShootTargetZoneTransport();
        rearIntakeShootInitiationLineTransport = new RearIntakeShootInititationLineTransport();

        //front intake shoot
        frontIntakeShootTrenchTransport = new FrontIntakeShootTrenchTransport();
        frontIntakeShootTargetZoneTransport = new FrontIntakeShootTargetZoneTransport();
        frontIntakeShootInitiationLineTransport = new FrontIntakeShootInitiationLineTransport();


        shootInitiationLineAndTransport = new ShootInitiationLineAndTransport();
        shootTargetZoneAndTransport = new ShootTargetZoneAndTransport();
        shootTrenchAndTransport = new ShootTrenchAndTransport();

        shootFromInitiationLine = new ShootFromInitiationLine();
        shootFromTargetZone = new ShootFromTargetZone();
        shootFromTrench = new ShootFromTrench();

        transportDeflectorOff = new TransportDeflectorOff();

        noPIDShootTrenchAndTransport = new NoPIDShootTrenchAndTransport();

        shootFromTargetZoneCompensate = new ShootFromTargetZoneCompensate();
        shootFromInitiationLineCompensate = new ShootFromInitiationLineCompensate();
        shootFromTrenchCompensate = new ShootFromTrenchCompensate();
        shootFromTrench = new ShootFromTrench();

        intakesAndTransportOff = new IntakesAndTransportOff();

    }

    public static RobotActionsCatalog getInstance() {
        if (instance == null) instance = new RobotActionsCatalog();
        return instance;
    }

    public AllSystemsOff getAllSystemsOff() {
        return allSystemsOff;
    }

    public FrontIntakeAndTransport getFrontIntakeAndTransport() {
        return frontIntakeAndTransport;
    }

    public IntakeFromFeederAndTransport getIntakeFromFeederAndTransport() {
        return intakeFromFeederAndTransport;
    }

    public RearIntakeAndShootBottomPort getRearIntakeAndShootBottomPort() {
        return rearIntakeAndShootBottomPort;
    }

    public RearIntakeAndTransport getRearIntakeAndTransport() {
        return rearIntakeAndTransport;
    }

    public RearIntakeShootTrenchTransport getRearIntakeShootTrenchTransport() {
        return rearIntakeShootTrenchTransport;
    }

    public RearIntakeShootTargetZoneTransport getRearIntakeShootTargetZoneTransport() {
        return rearIntakeShootTargetZoneTransport;
    }

    public RearIntakeShootInititationLineTransport getRearIntakeShootInitiationLineTransport() {
        return rearIntakeShootInitiationLineTransport;
    }

    public FrontIntakeShootTrenchTransport getFrontIntakeShootTrenchTransport() {
        return frontIntakeShootTrenchTransport;
    }

    public FrontIntakeShootTargetZoneTransport getFrontIntakeShootTargetZoneTransport() {
        return frontIntakeShootTargetZoneTransport;
    }

    public FrontIntakeShootInitiationLineTransport getFrontIntakeShootInitiationLineTransport() {
        return frontIntakeShootInitiationLineTransport;
    }

    public ShootInitiationLineAndTransport getShootInitiationLineAndTransport() {
        return shootInitiationLineAndTransport;
    }

    public ShootTargetZoneAndTransport getShootTargetZoneAndTransport() {
        return shootTargetZoneAndTransport;
    }

    public ShootTrenchAndTransport getShootTrenchAndTransport() {
        return shootTrenchAndTransport;
    }

    public ShootFromTrench getShootFromTrench() {
        return shootFromTrench;
    }

    public ShootFromInitiationLine getShootFromInitiationLine() {
        return shootFromInitiationLine;
    }

    public ShootFromTargetZone getShootFromTargetZone() {
        return shootFromTargetZone;
    }

    public TransportDeflectorOff getTransportDeflectorOff() {
        return transportDeflectorOff;
    }

    public NoPIDShootTrenchAndTransport getNoPIDShootTrenchAndTransport() {
        return noPIDShootTrenchAndTransport;
    }

    public ShooterOff getShooterOff() {
        return shooterOff;
    }

    public IntakesAndTransportOff getIntakesAndTransportOff() {
        return intakesAndTransportOff;
    }

    public ShootFromTrenchCompensate getShootFromTrenchCompensate() {
        return shootFromTrenchCompensate;
    }

    public ShootFromInitiationLineCompensate getShootFromInitiationLineCompensate() {
        return shootFromInitiationLineCompensate;
    }

    public ShootFromTargetZoneCompensate getShootFromTargetZoneCompensate() {
        return shootFromTargetZoneCompensate;
    }

    public FrontOutTakeAndTransport getFrontOutTakeAndTransport() {
        return frontOutTakeAndTransport;
    }

    public TransportationSystemShootingSpeed getTransportationSystemShootingSpeed() {
        return transportationSystemShootingSpeed;
    }
}