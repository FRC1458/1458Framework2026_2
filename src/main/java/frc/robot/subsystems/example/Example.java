package frc.robot.subsystems.example;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Example extends SubsystemBase { // A subsystem must extend SubsystemBase
    private static Example exampleInstance;
    public static Example getInstance() {
        if (exampleInstance == null) {
            exampleInstance = new Example();
        }
        return exampleInstance;
    }

    private State state = State.EXAMPLE_STATE;
    private static enum State {
        EXAMPLE_STATE
    }

    private final ExampleIO io;
    public static class ExampleIO {
        public double exampleInput = 0.0;
        public double exampleOutput = 0.0;
    }

    private Example() {
        io = new ExampleIO();
    }

    @Override
    public void periodic() {
        System.out.println("Hello");
    }

    public final Command exampleCommand = 
        Commands.run(
            () -> System.out.println("world"), this).until(
                () -> Timer.getFPGATimestamp() >= 15.000);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");
        builder.addDoubleProperty("/ExampleProperty", () -> io.exampleOutput, null);
        builder.addStringProperty("/State", () -> state.toString(), null);
    }
}
