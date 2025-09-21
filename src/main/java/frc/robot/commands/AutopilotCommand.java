package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.autopilot.Autopilot;
import frc.robot.lib.autopilot.Autopilot.*;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AutopilotCommand extends Command {
    private final Drive drive;
    private final APTarget target;
    private final Autopilot autopilot;
    private final ProfiledPIDVController thetaController;

    public AutopilotCommand(Pose2d target) {
        this(Drive.getInstance(), target);
    }

    public AutopilotCommand(Drive drive, Pose2d target) {
        this.drive = drive;
        this.target =  new APTarget(target);
        this.thetaController = new ProfiledPIDVController(Constants.Auto.ROTATION_CONSTANTS);
        autopilot = new Autopilot(
            new APProfile(
                new APConstraints(Constants.Drive.MAX_SPEED, Constants.Drive.MAX_ACCEL, 16))
                .withErrorTheta(Degrees.of(5))
                .withErrorXY(Centimeters.of(3))
                .withBeelineRadius(Centimeters.of(8)));
        setName("Autopilot to " + target.toString());
    }

    @Override
    public void execute() {
        var currentPose = RobotState.getLatestFieldToVehicle();
        var currentSpeeds = Util.fromTwist2d(RobotState.getSmoothedVelocity());
        var currentRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond, currentPose.getRotation());
        var result = autopilot.calculate(
            currentPose, currentRobotRelativeSpeeds, target.withEntryAngle(currentPose.getRotation()));

        SmartDashboard.putNumber("Autopilot/speedX", result.vx().in(MetersPerSecond));
        SmartDashboard.putNumber("Autopilot/speedY", result.vy().in(MetersPerSecond));
        
        thetaController.setTarget(result.targetAngle().getRadians());
        thetaController.setFeedforward(0.0);
        thetaController.setInput(
            new Pair<Double, Double>(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond));
        
        drive.setTargetSpeeds(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                result.vx(),
                result.vy(),
                Units.RadiansPerSecond.of(thetaController.getOutput().doubleValue()),
                currentPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return autopilot.atTarget(RobotState.getLatestFieldToVehicle(), target);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("autopilot ended");
        drive.setTargetSpeeds(new ChassisSpeeds());
    }
}
