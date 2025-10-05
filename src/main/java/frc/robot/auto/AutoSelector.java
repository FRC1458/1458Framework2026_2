package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A class to select autos
 */
public class AutoSelector {
    private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

    public AutoSelector() {
        chooser.setDefaultOption("None", () -> null);
        chooser.addOption("SnapTest", () -> AutoRoutines.testSnap());
        chooser.addOption("TrajectoryTest", () -> AutoRoutines.testTrajectoryAuto());
        chooser.addOption("TrajectoryTestOld", () -> AutoRoutines.driveAuto());
        chooser.addOption("AutopilotTestOld", () -> AutoRoutines.autopilotAuto());
        SmartDashboard.putData(chooser);
    }

    /** Gets the auto selected from the SmartDashboard */
    public Command getAuto() {
        return chooser.getSelected().get();
    }
}
