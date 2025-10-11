package frc.robot.auto;

import java.lang.reflect.Method;
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
        Method[] autos = AutoRoutines.class.getMethods();
        for (Method auto : autos) {
            var name = auto.getName();
            if (auto.getReturnType().equals(Command.class)) {
                chooser.addOption(name, () -> {
                    try{
                        return (Command) auto.invoke(null);
                    } catch (Exception e) {
                        return null;
                    }});
                // System.out.println(name);
            }
        }

        chooser.setDefaultOption("None", () -> null);
        SmartDashboard.putData(chooser);
    }

    /** Gets the auto selected from the SmartDashboard */
    public Command getAuto() {
        return chooser.getSelected().get();
    }
}
