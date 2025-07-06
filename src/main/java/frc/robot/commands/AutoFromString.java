package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

public class AutoFromString {
    public static class MutableInteger {
        public int x;
        public MutableInteger(int x) {
            this.x = x;
        }
    }
    
    public static Command parseSequential(String x) {
        String[] parts = x.split(" ");
        Command cmd = Commands.none();
        for (MutableInteger i = new MutableInteger(0); i.x < parts.length; i.x++) {
            try {
                cmd = cmd.andThen(getCommand(parts, i));
            } catch (Exception e) {
                DriverStation.reportWarning("Parse Error", true);
            }
        }
        return cmd;
    }

    public static Command parseParallel(String x) {
        String[] parts = x.split(" ");
        ArrayList<Command> cmds = new ArrayList<>();
        for (MutableInteger i = new MutableInteger(0); i.x < parts.length; i.x++) {
            try {
                cmds.add(getCommand(parts, i));
            } catch (Exception e) {
                DriverStation.reportWarning("Parse Error", true);
            }
        }
        return Commands.parallel((Command[]) cmds.toArray());
    }

    public static Command parseRace(String x) {
        String[] parts = x.split(" ");
        ArrayList<Command> cmds = new ArrayList<>();
        for (MutableInteger i = new MutableInteger(0); i.x < parts.length; i.x++) {
            try {
                cmds.add(getCommand(parts, i));
            } catch (Exception e) {
                DriverStation.reportWarning("Parse Error", true);
            }
        }
        return Commands.race((Command[]) cmds.toArray());
    }

    public static Command getCommand(String[] parts, MutableInteger index) {
        try {
            String part = parts[index.x];
            return switch (part) {
                case "Example" -> Commands.waitSeconds(1);
                /** Sequential Command Group */
                case "Seq{" -> {
                    String sub = "";
                    while ((part = parts[++index.x]) != "}") {
                        sub += part + " ";
                    }
                    yield parseSequential(sub);
                }
                /** Parallel Command Group */
                case "Par{" -> {
                    String sub = "";
                    while ((part = parts[++index.x]) != "}") {
                        sub += part + " ";
                    }
                    yield parseParallel(sub);
                }
                /** Race Command Group */
                case "Race{" -> {
                    String sub = "";
                    while ((part = parts[++index.x]) != "}") {
                        sub += part + " ";
                    }
                    yield parseRace(sub);
                }
                default -> Commands.none();
            };
        } catch (Exception e) {
            DriverStation.reportWarning("Parse Error", true);
            return Commands.none();
        }
    }
}
