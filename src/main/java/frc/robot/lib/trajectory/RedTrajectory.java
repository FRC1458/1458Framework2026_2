package frc.robot.lib.trajectory;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.localization.FieldUtil;

public class RedTrajectory {
    public TrajectoryType type;
    public static enum TrajectoryType {
        CHOREO,
        PATHPLANNER
    }

    public static class State {
        public double timestamp = 0.0;
        public Pose2d pose = Pose2d.kZero;
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public ChassisAccels accels = new ChassisAccels();

        /**
         * Creates a state at the origin with 0 speed and accel.
         */
        public State() {}

        /**
         * Creates a trajectory state.
         * @param timestamp The time in seconds.
         * @param pose The field-relative pose.
         * @param speeds The {@code ChassisSpeeds}, field-relative.
         * @param accels The {@code ChassisAccels}, field-relative.
         */
        public State(double timestamp, Pose2d pose, ChassisSpeeds speeds, ChassisAccels accels) {
            this.timestamp = timestamp;
            this.pose = pose;
            this.speeds = speeds;
            this.accels = accels;
        }

        /**
         * Creates a trajectory state from a {@code PathPlannerTrajectoryState}.
         * @param state The trajectory state.
         * @param nextState The next state (usually a very small time difference), used for calculating acceleration.
         */
        public State(PathPlannerTrajectoryState state, PathPlannerTrajectoryState nextState) {
            this(
                state.timeSeconds, 
                state.pose, 
                state.fieldSpeeds, 
                ChassisAccels.estimate(
                    state.fieldSpeeds, 
                    nextState.fieldSpeeds, 
                    MathUtil.clamp(nextState.timeSeconds - state.timeSeconds, 0.1, Double.POSITIVE_INFINITY)));
        }

        /**
         * Creates a trajectory state from a {@code PathPlannerTrajectoryState} with no acceleration.
         * @param state The trajectory state.
         */
        public State(PathPlannerTrajectoryState state) {
            this(
                state.timeSeconds, 
                state.pose, 
                state.fieldSpeeds, 
                new ChassisAccels());
        }

        /**
         * Creates a trajectory state from a Choreo trajectory state (a {@code SwerveSample}).
         * @param sample The trajectory state.
         */
        public State(SwerveSample sample) {
            this(
                sample.t,
                new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)),
                new ChassisSpeeds(sample.vx, sample.vy, sample.omega),
                new ChassisAccels(sample.ax, sample.ay, sample.alpha)  
            );
        }

        /**
         * Returns a new state, flipped along the current field symmetry. See {@link FieldUtil}.
         * @return The flipped state.
         */
        public State flip() {
            Pose2d flippedPose = FieldUtil.flipPose(pose);
            ChassisSpeeds flippedSpeeds = FieldUtil.flipSpeeds(speeds);
            ChassisAccels flippedAccels = FieldUtil.flipAccels(accels);
            return new State(timestamp, flippedPose, flippedSpeeds, flippedAccels);
        }

        /**
         * A class that represents the chassis acceleration.
         */
        public static class ChassisAccels {
            public final double ax, ay, alpha;

            /**
             * Creates a {@code ChassisAccels} with 0 acceleration.
             */
            public ChassisAccels() {
                ax = 0.0;
                ay = 0.0;
                alpha = 0.0;
            }

            /**
             * Creates a {@code ChassisAccels}.
             * @param ax The horizontal acceleration.
             * @param ay The vertical acceleration.
             * @param alpha The rotational acceleration.
             */
            public ChassisAccels(double ax, double ay, double alpha) {
                this.ax = ax;
                this.ay = ay;
                this.alpha = alpha;
            }

            /**
             * Estimates chassis acceleration from 2 {@code ChassisSpeeds}. This usually works better when {@code dt} is lower.
             * @param first The initial speeds.
             * @param second The final speeds.
             * @param dt The time difference.
             * @return The acceleration.
             */
            public static ChassisAccels estimate(ChassisSpeeds first, ChassisSpeeds second, double dt) {
                return new ChassisAccels(
                    (second.vxMetersPerSecond - first.vxMetersPerSecond) / dt,
                    (second.vyMetersPerSecond - first.vyMetersPerSecond) / dt,
                    (second.omegaRadiansPerSecond - first.omegaRadiansPerSecond) / dt
                );
            }
        }
    }
    
    public Trajectory<SwerveSample> mChoreoTrajectory;
    public PathPlannerTrajectory mPathplannerTrajectory;

    public double progress = 0.0;
    public final boolean flipped;

    /**
     * Creates a {@code RedTrajectory} from a Choreo trajectory. See {@link TrajectoryLoader}.
     * @param traj The Choreo trajectory.
     * @param flipForAlliance Whether to flip the trajectory based on the current alliance.
     */
    public RedTrajectory(Trajectory<SwerveSample> traj, boolean flipForAlliance) {
        mChoreoTrajectory = traj;
        this.type = TrajectoryType.CHOREO;
        this.flipped = flipForAlliance && (RobotState.getAlliance().get() == Alliance.Red);
    }

    /**
     * Creates a {@code RedTrajectory} from a PathPlanner trajectory. See {@link TrajectoryLoader}.
     * @param traj The PathPlanner trajectory.
     * @param flipForAlliance Whether to flip the trajectory based on the current alliance.
     */
    public RedTrajectory(PathPlannerTrajectory traj, boolean flipForAlliance) {
        mPathplannerTrajectory = traj;
        this.type = TrajectoryType.PATHPLANNER;
        this.flipped = flipForAlliance && (RobotState.getAlliance().get() == Alliance.Red);
    }

    /**
     * Gets the initial state.
     * @return The state at the beginning of the trajectory.
     */
    public State getInitialState() {
        return switch (type) {
            case CHOREO ->
                new State(mChoreoTrajectory.getInitialSample(false).get());
            case PATHPLANNER ->
                new State(mPathplannerTrajectory.getInitialState());
            default->
                new State();
        };
    }

    /**
     * Gets the final state.
     * @return The state at the end of the trajectory.
     */
    public State getFinalState() {
        return switch (type) {
            case CHOREO ->
                new State(mChoreoTrajectory.getFinalSample(false).get());
            case PATHPLANNER ->
                new State(mPathplannerTrajectory.getEndState());
            default ->
                new State();
        };
    }

    /**
     * Advances the trajectory by a time.
     * @param seconds The time to advance by, in seconds.
     * @return The state at the current time.
     */
    public State advance(double seconds) {
        this.progress += seconds;
        if (this.type == TrajectoryType.CHOREO) {
            this.progress = MathUtil.clamp(progress, 0.0, mChoreoTrajectory.getTotalTime());
        } else if (this.type == TrajectoryType.PATHPLANNER) {
            this.progress = MathUtil.clamp(progress, 0.0, mPathplannerTrajectory.getTotalTimeSeconds());
        }
        return sample(this.progress);
    }

    /**
     * Advances the trajectory to a time.
     * @param seconds The time to advance to, in seconds.
     * @return The state at the current time.
     */
    public State advanceTo(double seconds) {
        this.progress = seconds;
        if (this.type == TrajectoryType.CHOREO) {
            this.progress = MathUtil.clamp(progress, 0.0, mChoreoTrajectory.getTotalTime());
        } else if (this.type == TrajectoryType.PATHPLANNER) {
            this.progress = MathUtil.clamp(progress, 0.0, mPathplannerTrajectory.getTotalTimeSeconds());
        }
        return sample(this.progress);
    }

    /**
     * Gets the current state.
     * @return The state at the current time.
     */
    public State getCurrent() {
        return sample(this.progress);
    }

    /**
     * Checks if the trajectory has reached the end.
     * @return Whether the trajectory is done.
     */
    public boolean isDone() {
        switch (type) {
            case PATHPLANNER:
                return mPathplannerTrajectory.getTotalTimeSeconds() <= progress;
            case CHOREO:
                return mChoreoTrajectory.getTotalTime() <= progress;
            default:
                return true;
        }
    }

    /**
     * Gets the state at the timestamp.
     * @param seconds The time to view.
     * @return The state at the timestamp.
     */
    public State sample(double timestamp) {
        switch (type) {
            case PATHPLANNER:
                State state = new State(
                    mPathplannerTrajectory.sample(timestamp),
                    mPathplannerTrajectory.sample(
                        MathUtil.clamp(
                            timestamp + Constants.DT,
                            0.0, 
                            mPathplannerTrajectory.getTotalTimeSeconds())));
                return flipped ? state.flip() : state;
            case CHOREO:
                State state2 = new State(
                    mChoreoTrajectory.sampleAt(timestamp, false).get());
                return flipped ? state2.flip() : state2;
            default:
                return new State();
        }
    }
}
