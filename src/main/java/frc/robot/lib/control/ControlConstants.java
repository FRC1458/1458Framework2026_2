package frc.robot.lib.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A class containing controller constants.
 */
public class ControlConstants {
    /**
     * Constants used for a {@link PIDController}
     */
    public static class PIDConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public PIDConstants(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }
    }

    /**
     * Constants used for a {@link PIDFController}
     */
    public static class PIDFConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public PIDFConstants(double p, double i, double d, double f) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.kF = f;
        }
        public PIDFConstants(PIDConstants constants) {
            this.kP = constants.kP;
            this.kI = constants.kI;
            this.kD = constants.kD;
            this.kF = 0.0;
        }
    }
    
    /**
     * Constants used for a {@link ProfiledPIDFController}
     */
    public static class ProfiledPIDFConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final TrapezoidProfile.Constraints constraints;
        public ProfiledPIDFConstants(double p, double i, double d, double f, TrapezoidProfile.Constraints constraints) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.kF = f;
            this.constraints = constraints;
        }
        public ProfiledPIDFConstants(PIDConstants constants, TrapezoidProfile.Constraints constraints) {
            this.kP = constants.kP;
            this.kI = constants.kI;
            this.kD = constants.kD;
            this.kF = 0.0;
            this.constraints = constraints;
        }
    }
}
