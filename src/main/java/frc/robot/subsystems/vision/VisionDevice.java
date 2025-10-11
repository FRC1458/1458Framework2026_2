package frc.robot.subsystems.vision;

import frc.robot.lib.localization.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Limelight.VisionDeviceConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionDevice {
	private final VisionDeviceConstants mConstants;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	public Field2d robotField;
	public PhotonCamera mCamera;
	public PhotonPoseEstimator mPoseEstimator;
	private boolean inSnapRange;
	private boolean hasTarget;

	public Pose2d botPose;

	public VisionDevice(VisionDeviceConstants constants) {
		robotField = new Field2d();
		SmartDashboard.putData("VisionDevice/" + constants.tableName, robotField);

		mCamera = new PhotonCamera(constants.tableName);
		mPoseEstimator = new PhotonPoseEstimator(FieldLayout.APRILTAG_MAP, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, constants.robotToCamera);

		mConstants = constants;

		inSnapRange = false;
		hasTarget = false;
	}

	private void processFrames() {
		var result = mCamera.getLatestResult();
		hasTarget = result.hasTargets();

		if (!hasTarget) {
			return; 
		} else {
			var target = result.getBestTarget();
			var initBotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), FieldLayout.APRILTAG_MAP.getTagPose(target.getFiducialId()).get(), mConstants.robotToCamera.inverse());
			var estimatedPose = mPoseEstimator.update(result);

			if (estimatedPose.isEmpty()) botPose = initBotPose.toPose2d();
			else botPose = estimatedPose.get().estimatedPose.toPose2d();

			Drive.getInstance().getCtreDrive().addVisionMeasurement(botPose, result.getTimestampSeconds());
			mPoseEstimator.setReferencePose(Drive.getInstance().getPose());

			robotField.setRobotPose(botPose);
			SmartDashboard.putData("VisionDevice/" + mConstants.tableName, robotField);
		};

		int[] validIds = { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 };

		if (result.getBestTarget().getAlternateCameraToTarget().getTranslation().getNorm() < 3
				&& MathUtil.inputModulus(result.getBestTarget().getAlternateCameraToTarget().getRotation().toRotation2d().getDegrees() + 15, -180, 180) < 30
				&& Arrays.stream(validIds).anyMatch(n -> n == (int) result.getBestTarget().getFiducialId())) {
			inSnapRange = true;
		} else {
			inSnapRange = false;
		}
	}

	public boolean inSnapRange() {
		return inSnapRange;
	}

	public boolean hasTarget() {
		return hasTarget;
	}

	public void periodic() {
		mPeriodicIO.is_connected = !(Timer.getFPGATimestamp() - mPeriodicIO.latest_timestamp > 1.0);

		processFrames();

		SmartDashboard.putNumber(
				"Vision " + mConstants.tableName + "/Last Update Timestamp Timestamp", mPeriodicIO.latest_timestamp);
		SmartDashboard.putNumber("Vision " + mConstants.tableName + "/N Queued Updates", mPeriodicIO.frames.size());
		SmartDashboard.putBoolean("Vision " + mConstants.tableName + "/is Connnected", mPeriodicIO.is_connected);
	}

	public boolean isConnected() {
		return mPeriodicIO.is_connected;
	}

	public static class PeriodicIO {
		// inputs
		double camera_exposure = 20;
		boolean camera_auto_exposure = false;
		double camera_gain = 10;

		// Outputs
		long fps = -1;
		double latest_timestamp = 0.0;
		List<VisionFrame> frames = new ArrayList<VisionFrame>();
		boolean is_connected;
	}

	private static class VisionFrame implements Comparable<VisionFrame> {
		double timestamp;

		@Override
		public int compareTo(VisionFrame o) {
			return Double.compare(timestamp, o.timestamp);
		}
	}
}