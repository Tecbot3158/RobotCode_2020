package frc.robot;

import frc.robot.commands.robotActions.AllSystemsOff;
import frc.robot.commands.robotActions.intakeTransport.RearIntakeAndShootBottomPort;
import frc.robot.commands.robotActions.intakeTransport.FrontIntakeAndTransport;
import frc.robot.commands.robotActions.intakeTransport.IntakeFromFeederAndTransport;
import frc.robot.commands.robotActions.intakeTransport.RearIntakeAndTransport;
import frc.robot.commands.robotActions.mixed.*;
import frc.robot.commands.robotActions.shootTransport.ShootInitiationLineAndTransport;
import frc.robot.commands.robotActions.shootTransport.ShootTargetZoneAndTransport;
import frc.robot.commands.robotActions.shootTransport.ShootTrenchAndTransport;

public class RobotActionsCatalog {
    private static RobotActionsCatalog instance;

    private AllSystemsOff allSystemsOff;

    private FrontIntakeAndTransport frontIntakeAndTransport;
    private IntakeFromFeederAndTransport intakeFromFeederAndTransport;
    private RearIntakeAndShootBottomPort rearIntakeAndShootBottomPort;
    private RearIntakeAndTransport rearIntakeAndTransport;

    private RearIntakeShootTrenchTransport rearIntakeShootTrenchTransport;
    private RearIntakeShootTargetZoneTransport rearIntakeShootTargetZoneTransport;
    private RearIntakeShootInititationLineTransport rearIntakeShootInititationLineTransport;

    private FrontIntakeShootTrenchTransport frontIntakeShootTrenchTransport;
    private FrontIntakeShootTargetZoneTransport frontIntakeShootTargetZoneTransport;
    private FrontIntakeShootInitiationLineTransport frontIntakeShootInitiationLineTransport;

    private ShootInitiationLineAndTransport shootInitiationLineAndTransport;
    private ShootTargetZoneAndTransport shootTargetZoneAndTransport;
    private ShootTrenchAndTransport shootTrenchAndTransport;


    public RobotActionsCatalog() {

        allSystemsOff = new AllSystemsOff();

        frontIntakeAndTransport = new FrontIntakeAndTransport();
        intakeFromFeederAndTransport = new IntakeFromFeederAndTransport();
        rearIntakeAndShootBottomPort = new RearIntakeAndShootBottomPort();
        rearIntakeAndTransport = new RearIntakeAndTransport();

        //rear intake shoot
        rearIntakeShootTrenchTransport = new RearIntakeShootTrenchTransport();
        rearIntakeShootTargetZoneTransport = new RearIntakeShootTargetZoneTransport();
        rearIntakeShootInititationLineTransport = new RearIntakeShootInititationLineTransport();

        //front intake shoot
        frontIntakeShootTrenchTransport = new FrontIntakeShootTrenchTransport();
        frontIntakeShootTargetZoneTransport = new FrontIntakeShootTargetZoneTransport();
        frontIntakeShootInitiationLineTransport = new FrontIntakeShootInitiationLineTransport();


        shootInitiationLineAndTransport = new ShootInitiationLineAndTransport();
        shootTargetZoneAndTransport = new ShootTargetZoneAndTransport();
        shootTrenchAndTransport = new ShootTrenchAndTransport();

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

    public RearIntakeShootInititationLineTransport getRearIntakeShootInititationLineTransport() {
        return rearIntakeShootInititationLineTransport;
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
}