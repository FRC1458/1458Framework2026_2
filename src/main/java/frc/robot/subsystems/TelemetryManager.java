package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

    private final ArrayList<StructArrayPublisherEntry<?>> structArrayPublishers = new ArrayList<>();

    public static record StructArrayPublisherEntry<T> (
        StructArrayPublisher<T> publisher, Supplier<T[]> supplier
    ) {
        public void publish() {
            publisher.set(supplier.get());
        }
    }


    private final ArrayList<StructPublisherEntry<?>> structPublishers;

    public static record StructPublisherEntry<T> (
        StructPublisher<T> publisher, Supplier<T> supplier
    ) {
        public void publish() {
            publisher.set(supplier.get());
        }
    }

    private final ArrayList<Sendable> sendables;

    private TelemetryManager() {
        structPublishers = new ArrayList<>();
        sendables = new ArrayList<>();
    }

    @Override
    public void periodic() {
        for (StructArrayPublisherEntry<?> entry : structArrayPublishers) {
            entry.publish();
        }

        for (StructPublisherEntry<?> entry : structPublishers) {
            entry.publish();
        }
    }
    
    public <T> void addStructPublisher(String name, Struct<T> struct, Supplier<T> supplier) {
        structPublishers.add(
            new StructPublisherEntry<T>(
                NetworkTableInstance.getDefault()
                    .getStructTopic("SmartDashboard/" + name, struct).publish(), 
                supplier));
    }

    public <T> void addStructArrayPublisher(String name, Struct<T> struct, Supplier<T[]> supplier) {
        structArrayPublishers.add(
            new StructArrayPublisherEntry<>(
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
