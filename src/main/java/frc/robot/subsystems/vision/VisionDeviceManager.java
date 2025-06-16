package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.lib.util.TunableNumber;
import frc.robot.lib.util.MovingAverage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class VisionDeviceManager extends SubsystemBase {
    public static boolean enabled;
	private static VisionDeviceManager mInstance;

	public static VisionDeviceManager getInstance() {
		if (mInstance == null && enabled) {
			mInstance = new VisionDeviceManager();
		}
		return mInstance;
	}

	private VisionDevice mLeftCamera;
	private VisionDevice mRightCamera;
	private VisionDevice mFrontCamera;
	private VisionDevice mBackCamera;

	private List<VisionDevice> mAllCameras;

	private static TunableNumber timestampOffset = new TunableNumber("VisionTimestampOffset", (0.1), false);

	private MovingAverage<Double> mHeadingAvg = new MovingAverage<Double>(
		100, Double.valueOf(0), 
		(Double x, Double y) -> { return x + y; }, (Double x, Integer y) -> { return x / y; });
	private double mMovingAvgRead = 0.0;

	private static boolean disable_vision = false;

	private VisionDeviceManager() {
		mLeftCamera = new VisionDevice(Constants.Limelight.L_CONSTANTS);
		mRightCamera = new VisionDevice(Constants.Limelight.R_CONSTANTS);
		mFrontCamera = new VisionDevice(Constants.Limelight.F_CONSTANTS);
		mBackCamera = new VisionDevice(Constants.Limelight.B_CONSTANTS);
		mAllCameras = List.of(mLeftCamera, mRightCamera, mFrontCamera, mBackCamera);
	}

	@Override
	public void periodic() {
		mAllCameras.forEach(VisionDevice::periodic);
		mMovingAvgRead = mHeadingAvg.getAverage();
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAverageRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
	}

	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public synchronized MovingAverage<Double> getMovingAverage() {
		return mHeadingAvg;
	}

	public synchronized boolean fullyConnected() {
		return mLeftCamera.isConnected()
			&& mRightCamera.isConnected()
			&& mFrontCamera.isConnected()
			&& mBackCamera.isConnected();
	}

	public synchronized boolean inRange () {
		return mFrontCamera.inSnapRange() && mFrontCamera.hasTarget();
	}

	public synchronized VisionDevice getLeftVision() {
		return mLeftCamera;
	}

	public synchronized VisionDevice getRightVision() {
		return mRightCamera;
	}

	public synchronized VisionDevice getFrontVision() {
		return mFrontCamera;
	}

	public synchronized VisionDevice getBackVision() {
		return mBackCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset.get();
	}

	public static boolean visionDisabled() {
		return disable_vision;
	}

	public static void setDisableVision(boolean disable) {
		disable_vision = disable;
	}
}
