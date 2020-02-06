/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robotActions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AllSystemsOff extends SequentialCommandGroup {
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
    public AllSystemsOff() {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super();
        addCommands(
                /*
                FI PNEUMATICS OFF
                FI OFF MODE
                RI PNEUMATICS OFF
                RI OFF MODE
                PCTS OFF, DEF OFF
                PCS OFF
                 */
                new InstantCommandExample()
        );
    }
}

class InstantCommandExample extends InstantCommand {
    @Override
    public void initialize(){
        System.out.println("ALL SYSTEMS OFF");
    }

}