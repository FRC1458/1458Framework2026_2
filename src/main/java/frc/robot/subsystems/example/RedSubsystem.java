package frc.robot.subsystems.example;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class RedSubsystem extends SubsystemBase { 
    private static interface State {
        public String getName();
        public Command getCommand();
    }

    public static abstract class IO {
        
    }

    private RedSubsystem() {
        super();
    }

    public abstract State getState();
    protected abstract IO getIO();
    public abstract Command getRunningCommand();

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("/State", () -> getState().getName(), null);
    }
}
