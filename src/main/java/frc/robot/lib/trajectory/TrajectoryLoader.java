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
    private TrajectoryLoader() {}

    public static Optional<RedTrajectory> loadTrajectory(RedTrajectory.TrajectoryType type, String fileName) {
        switch(type) {
            case CHOREO:                
                try {
                    return Optional.of(new RedTrajectory((Trajectory<SwerveSample>) Choreo.loadTrajectory(fileName).get()));
                } catch (Exception e) {
                    DriverStation.reportError(fileName + " is not a valid trajectory!", e.getStackTrace());
                }
                break;
            case PATHPLANNER:
                try {
                    return Optional.of(new RedTrajectory(PathPlannerPath.fromPathFile(fileName).getIdealTrajectory(Constants.Pathplanner.config).get()));
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
