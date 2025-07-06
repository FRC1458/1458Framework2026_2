package frc.robot.lib.trajectory;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class TrajectoryLoader {
    /** Static class */
    private TrajectoryLoader() {}

    /**
     * Loads a trajectory from the deploy directory.
     * @param type The type of trajectory to load.
     * @param fileName The file name (without the extension).
     * @return The trajectory.
     */
    public static Optional<RedTrajectory> loadAutoTrajectory(RedTrajectory.TrajectoryType type, String fileName) {
        switch(type) {
            case CHOREO:                
                // This is such a monstrosity but i can live with it
                Optional<? extends Trajectory<?>> load = Choreo.loadTrajectory(fileName);
                if (load.isPresent()) {
                    Trajectory<?> trajectory = load.get();
                    if (trajectory.samples().get(0) instanceof SwerveSample) {
                        @SuppressWarnings("unchecked")
                        Trajectory<SwerveSample> casted = (Trajectory<SwerveSample>) trajectory;
                        return Optional.of(new RedTrajectory(casted, true));
                    } else {
                        DriverStation.reportWarning(fileName + " is not a swerve trajectory!", false);
                        return Optional.empty();
                    }
                } else {
                    DriverStation.reportWarning(fileName + " is not a valid trajectory!", false);
                    return Optional.empty();
                }
            case PATHPLANNER:
                try {
                    PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);
                    Optional<PathPlannerTrajectory> traj = path.getIdealTrajectory(Constants.Pathplanner.config);
                    if (traj.isPresent()) {
                        return Optional.of(new RedTrajectory(traj.get(), true, path.name));
                    } else {
                        DriverStation.reportWarning(fileName + " is not a valid trajectory!", false);
                        return Optional.empty();
                    }
                } catch (Exception e) {
                    DriverStation.reportWarning(fileName + " is not a valid trajectory! " + e.getClass().getSimpleName(), false);
                    return Optional.empty();
                }
            default:
                DriverStation.reportWarning(type.name() + " is not a trajectory type!", false);
                return Optional.empty();
        }
    }
}
