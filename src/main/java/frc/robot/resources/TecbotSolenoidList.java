package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SolenoidBase;

import java.util.ArrayList;
import java.util.List;

public class TecbotSolenoidList {
    List<SolenoidBase> solenoidBaseList;

    public TecbotSolenoidList(boolean[] isDoubleSolenoid, int[]... ports) {
        solenoidBaseList = new ArrayList<>();
        if (ports.length != isDoubleSolenoid.length) {
            DriverStation.reportError("Amount of ports is not equal to amount of solenoid types. FROM: TecbotSolenoidList", true);
            return;
        }
        //for(int i = 0; sol)

    }
}
