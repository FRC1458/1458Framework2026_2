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

	private MovingAverage mHeadingAvg = new MovingAverage(100);
	private double mMovingAvgRead = 0.0;

	private static boolean disable_vision = false;

	private VisionDeviceManager() {
		mLeftCamera = new VisionDevice(Constants.Limelight.LEFT_VISION_DEVICE);
		mRightCamera = new VisionDevice(Constants.Limelight.RIGHT_VISION_DEVICE);
		mFrontCamera = new VisionDevice(Constants.Limelight.FRONT_VISION_DEVICE);
		mBackCamera = new VisionDevice(Constants.Limelight.BACK_VISION_DEVICE);
		mAllCameras = List.of(mLeftCamera, mRightCamera, mFrontCamera, mBackCamera);
	}

	@Override
	public void periodic() {
		//System.out.println("VisionDevice.readPeriodic()");
		mAllCameras.forEach(VisionDevice::periodic);
		mMovingAvgRead = mHeadingAvg.getAverage();
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAverageRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
	}

	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public synchronized MovingAverage getMovingAverage() {
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
