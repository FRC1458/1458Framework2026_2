package frc.robot.lib.trajectory;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
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

        public State() {}

        public State(double timestamp, Pose2d pose, ChassisSpeeds speeds, ChassisAccels accels) {
            this.timestamp = timestamp;
            this.pose = pose;
            this.speeds = speeds;
            this.accels = accels;
        }

        public State(PathPlannerTrajectoryState state, PathPlannerTrajectoryState nextState) {
            this(
                state.timeSeconds, 
                state.pose, 
                state.fieldSpeeds, 
                ChassisAccels.estimate(
                    state.fieldSpeeds, 
                    nextState.fieldSpeeds, 
                    nextState.timeSeconds - state.timeSeconds
                )
            );
        }

        public State(PathPlannerTrajectoryState state) {
            this(
                state.timeSeconds, 
                state.pose, 
                state.fieldSpeeds, 
                new ChassisAccels()
            );
        }

        public State(SwerveSample sample) {
            this(
                sample.t,
                new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)),
                new ChassisSpeeds(sample.vx, sample.vy, sample.omega),
                new ChassisAccels(sample.ax, sample.ay, sample.alpha)  
            );
        }

        public State flip() {
            Pose2d flippedPose = FieldUtil.flipPose(pose);
            ChassisSpeeds flippedSpeeds = FieldUtil.flipSpeeds(speeds);
            ChassisAccels flippedAccels = FieldUtil.flipAccels(accels);
            return new State(timestamp, flippedPose, flippedSpeeds, flippedAccels);
        }

        public static class ChassisAccels {
            public final double ax, ay, alpha;

            public ChassisAccels() {
                ax = 0.0;
                ay = 0.0;
                alpha = 0.0;
            }

            public ChassisAccels(double ax, double ay, double alpha) {
                this.ax = ax;
                this.ay = ay;
                this.alpha = alpha;
            }

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

    public RedTrajectory(Trajectory<SwerveSample> traj) {
        mChoreoTrajectory = traj;
        this.type = TrajectoryType.CHOREO;
    }

    public RedTrajectory(PathPlannerTrajectory traj) {
        mPathplannerTrajectory = traj;
        this.type = TrajectoryType.PATHPLANNER;
    }

    public State getInitialState() {
        switch(type) {
            case CHOREO:
                return new State(mChoreoTrajectory.getInitialSample(false).get());
            case PATHPLANNER:
                return new State(mPathplannerTrajectory.getInitialState());
            default:
                return new State();
        }
    }

    public State getFinalState() {
        switch(type) {
            case CHOREO:
                return new State(mChoreoTrajectory.getFinalSample(false).get());
            case PATHPLANNER:
                return new State(mPathplannerTrajectory.getEndState());
            default:
                return new State();
        }
    }

    public State advance(double seconds) {
        this.progress += seconds;
        if (this.type == TrajectoryType.CHOREO) {
            this.progress = MathUtil.clamp(progress, 0.0, mChoreoTrajectory.getTotalTime());
        } else if (this.type == TrajectoryType.PATHPLANNER) {
            this.progress = MathUtil.clamp(progress, 0.0, mPathplannerTrajectory.getTotalTimeSeconds());
        }
        return sample(this.progress);
    }

    public State advanceTo(double seconds) {
        this.progress = seconds;
        if (this.type == TrajectoryType.CHOREO) {
            this.progress = MathUtil.clamp(progress, 0.0, mChoreoTrajectory.getTotalTime());
        } else if (this.type == TrajectoryType.PATHPLANNER) {
            this.progress = MathUtil.clamp(progress, 0.0, mPathplannerTrajectory.getTotalTimeSeconds());
        }
        return sample(this.progress);
    }

    public State getCurrent() {
        return sample(this.progress);
    }

    public boolean getIsDone() {
        switch(type) {
            case PATHPLANNER:
                return mPathplannerTrajectory.getTotalTimeSeconds() <= progress;
            case CHOREO:
                return mChoreoTrajectory.getTotalTime() <= progress;
            default:
                return true;
        }
    }

    public State sample(double timestamp) {
        switch(type) {
            case PATHPLANNER:
                return new State(
                    mPathplannerTrajectory.sample(timestamp),
                    mPathplannerTrajectory.sample(
                        MathUtil.clamp(
                            timestamp + Constants.DT,
                            0.0, 
                            mPathplannerTrajectory.getTotalTimeSeconds()
                            )
                        )
                );
            case CHOREO:
                return new State(
                    mChoreoTrajectory.sampleAt(timestamp, false).get()
                );
            default:
                return new State();
        }
    }
}
