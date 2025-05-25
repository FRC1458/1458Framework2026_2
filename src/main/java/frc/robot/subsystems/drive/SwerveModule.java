package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.swerve.SwerveModuleConstants;

public class SwerveModule {
    private SwerveModuleConstants mConstants;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private static class PeriodicIO {
        SwerveModuleState targetState = new SwerveModuleState();
    }

    public SwerveModule(SwerveModuleConstants constants) {
        mConstants = constants;
    }

    public void setTargetState(SwerveModuleState state) {
        mPeriodicIO.targetState = state;
    }
}
