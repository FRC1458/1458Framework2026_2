package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetryManager extends SubsystemBase {
    private static TelemetryManager telemetryManagerInstance;
    public static TelemetryManager getInstance() {
        if (telemetryManagerInstance == null) {
            telemetryManagerInstance = new TelemetryManager();
        }
        return telemetryManagerInstance;
    }

    private final ArrayList<Pair<StructArrayPublisher<?>, Supplier<?>>> publishers;
    private final ArrayList<Sendable> sendables;

    private TelemetryManager() {
        publishers = new ArrayList<>();
        sendables = new ArrayList<>();
    }
    
    public <T> void addPublisher(String name, Struct<T> struct, Supplier<T> supplier) {
        publishers.add(
            new Pair<>(
                NetworkTableInstance.getDefault()
                    .getStructArrayTopic("SmartDashboard/" + name, struct).publish(), 
                    supplier));
    }

    public void addSendable(Sendable sendable) {
        sendables.add(sendable);
        SmartDashboard.putData(sendable);
    }

    public void addNumber(Number number) {}
}
