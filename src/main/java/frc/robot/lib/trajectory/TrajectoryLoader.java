package frc.robot.lib.trajectory;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.util.FileLogger;
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
                try {
                    return Optional.of(new RedTrajectory((Trajectory<SwerveSample>) Choreo.loadTrajectory(fileName).get(), true));
                } catch (Exception e) {
                    DriverStation.reportError(fileName + " is not a valid trajectory!", e.getStackTrace());
                }
                break;
            case PATHPLANNER:
                try {
                    var other = PathPlannerPath.fromPathFile(fileName)
                    .getIdealTrajectory(Constants.Pathplanner.config)
                    .get();
                    System.out.println(other);
                    return Optional.of(new RedTrajectory(PathPlannerPath.fromPathFile(fileName).getIdealTrajectory(Constants.Pathplanner.config).get(), true));
                } catch (Exception e) {
                    DriverStation.reportError(fileName + " is not a valid trajectory!", e.getStackTrace());
                }
                break;
            default:
                DriverStation.reportWarning(type.name() + " is not a trajectory type!", false);
                return Optional.empty();
        }
        DriverStation.reportWarning("idk something happened man", false);
        return Optional.empty();
    }
}
