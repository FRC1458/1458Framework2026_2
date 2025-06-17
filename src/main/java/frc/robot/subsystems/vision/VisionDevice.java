package frc.robot.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import frc.robot.Constants.Limelight.VisionDeviceConstants;
import frc.robot.Constants.Limelight.VisionDeviceConstants;

import static frc.robot.subsystems.drive.Pigeon.mPigeon;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionUpdate;
import frc.robot.lib.localization.FieldLayout;
import frc.robot.lib.localization.LimelightHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.Vector;

public class VisionDevice {
	private final VisionDeviceConstants mConstants;
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private NetworkTable mConfigTable;
	private NetworkTable mOutputTable;
	private NetworkTable mCalibTable;

	private IntegerSubscriber mVisible;
	private DoubleArraySubscriber mObservations;
	private DoubleArraySubscriber mStdDevs;
	private IntegerSubscriber mFPS;
	private IntegerSubscriber mID;

	public Field2d robotField;
	private boolean inSnapRange;
	private boolean hasTarget;

	public VisionDevice(VisionDeviceConstants constants) {
		robotField = new Field2d();
		SmartDashboard.putData("VisionDevice/" + constants.tableName, robotField);

		mConstants = constants;
		mConfigTable = NetworkTableInstance.getDefault().getTable(mConstants.tableName + "/configs");
		mCalibTable = NetworkTableInstance.getDefault().getTable(mConstants.tableName + "/calibration");
		mOutputTable = NetworkTableInstance.getDefault().getTable(mConstants.tableName);

		mVisible = mOutputTable
				.getIntegerTopic("tv")
				.subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mObservations = mOutputTable
				.getDoubleArrayTopic("botpose_orb_wpiblue")
				.subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mStdDevs = mOutputTable
				.getDoubleArrayTopic("stddevs")
				.subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mFPS = mOutputTable
				.getIntegerTopic("fps")
				.subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mID = mOutputTable
				.getIntegerTopic("tid")
				.subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

		mConfigTable.getDoubleTopic("fiducial_size_m").publish().set(FieldLayout.APRITAG_WIDTH);
		try {
			mConfigTable
					.getStringTopic("tag_layout")
					.publish()
					.set(new ObjectMapper().writeValueAsString(FieldLayout.APRILTAG_MAP));
		} catch (JsonProcessingException e) {
			throw new RuntimeException("Failed to serialize apriltag layout");
		}

		mConfigTable.getEntry("camera_exposure").setDouble(mPeriodicIO.camera_exposure);
		mConfigTable.getEntry("camera_auto_exposure").setDouble(mPeriodicIO.camera_auto_exposure ? 0.0 : 1.0);
		mConfigTable.getEntry("camera_gain").setDouble(mPeriodicIO.camera_gain);
	
		inSnapRange = false;
		hasTarget = false;
	}

	private void processFrames() {
		if (mVisible.get() == 0) {
			hasTarget = false;
			return;
		} else {
			hasTarget = true;
		}

		double[] mt2Pose = mObservations.get();
		double[] stdDevs = mStdDevs.get();
		double timestamp = Timer.getFPGATimestamp() * Math.pow(10, 6) - VisionDeviceManager.getTimestampOffset();

		if (mt2Pose.length == 0) {
			hasTarget = false;
			return;
		}

		LimelightHelpers.SetRobotOrientation(mConstants.tableName, mPigeon.getYaw().getDegrees(), 0, 0, 0, 0, 0);

		Pose2d botPose = new Pose2d(mt2Pose[0], mt2Pose[1], new Rotation2d(mt2Pose[5] * Math.PI / 180));
		Vector<N2> stdDevsVec = VecBuilder.fill(stdDevs[6], stdDevs[7]);

		robotField.setRobotPose(botPose);
		Pose2d targetSpace_pose = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(mConstants.tableName));
		//Vector<N2> betterDevs = VecBuilder.fill(0.005 * targetSpace_pose.getX(), 0.005 * targetSpace_pose.getY());

		RobotState.addVisionUpdate(
			new VisionUpdate(
					timestamp,
					botPose.getTranslation(),
					new Translation2d(0, 0),
					stdDevsVec));
		
		//Pose2d targetSpace_pose = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(mConstants.kTableName));
		int[] validIds = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

		if (
			targetSpace_pose.getTranslation().getDistance(new Translation2d(0, 0)) < 3 
			&& MathUtil.inputModulus(targetSpace_pose.getRotation().getDegrees() + 15, -180, 180) < 30
			&& Arrays.stream(validIds).anyMatch(n->n==(int) mID.get())
		) {
			// System.out.println(mID.get());
			inSnapRange = true;
		} else {
			// System.out.println(MathUtil.inputModulus(mPigeon.getYaw().getDegrees(), -180, 180));
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
		mPeriodicIO.fps = mFPS.get();

		mPeriodicIO.is_connected = !(Timer.getFPGATimestamp() - mPeriodicIO.latest_timestamp > 1.0);

		processFrames();

		SmartDashboard.putNumber(
				"Vision " + mConstants.tableName + "/Last Update Timestamp Timestamp", mPeriodicIO.latest_timestamp);
		SmartDashboard.putNumber("Vision " + mConstants.tableName + "/N Queued Updates", mPeriodicIO.frames.size());
		SmartDashboard.putBoolean("Vision " + mConstants.tableName + "/is Connnected", mPeriodicIO.is_connected);
	}

	public void captureCalibrationFrame() {
		mCalibTable.getEntry("wants_frames").setBoolean(true);
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

	private static class VisionFrame {
		double timestamp;
		double[] frame_data;
		double[] stdDevs;
	
		// For comparator
		public double getTimestamp() {
			return timestamp;
		}
	}
}
