package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants.Ports;
import frc.robot.lib.drivers.CanDeviceId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class Cancoders {
	public static final Cancoders mCanCoders = new Cancoders();

	private final CANcoder mFrontLeft;
	private final CANcoder mFrontRight;
	private final CANcoder mBackLeft;
	private final CANcoder mBackRight;

	private final CanTimeObserver mFrontRightObserver;
	private final CanTimeObserver mFrontLeftObserver;
	private final CanTimeObserver mBackLeftObserver;
	private final CanTimeObserver mBackRightObserver;

	private static final double BOOT_UP_ERROR_ALLOWANCE_TIME = 10.0;

	private static class CanTimeObserver {
		private final CANcoder cancoder;
		private Optional<Double> lastTimestamp = Optional.empty();
		private int validUpdates = 0;
		private static final int kRequiredValidTimestamps = 10;

		public CanTimeObserver(CANcoder cancoder) {
			this.cancoder = cancoder;
		}

		public boolean hasUpdate() {
			StatusSignal<Angle> absolutePositionSignal = cancoder.getAbsolutePosition();

			double timestamp = absolutePositionSignal.getTimestamp().getTime();
			if (lastTimestamp.isEmpty()) {
				lastTimestamp = Optional.of(timestamp);
			}
			if (timestamp > lastTimestamp.get()) {
				validUpdates++;
				lastTimestamp = Optional.of(timestamp);
			}
			return validUpdates > kRequiredValidTimestamps;
		}
	}

	private CANcoder build(Constants.Ports canDeviceId) {
		CANcoder thisCancoder = new CANcoder(canDeviceId.id, canDeviceId.bus);
		CANcoderConfigurator configurator = thisCancoder.getConfigurator();
		CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

		canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; 
		canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
		canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		double startTime = Timer.getFPGATimestamp();
		boolean timedOut = false;
		boolean goodInit = false;
		int attempt = 1;
		while (!goodInit && !timedOut && attempt < 20) {
			System.out.println("Initing CANCoder " + canDeviceId.id + " / attempt: " + attempt + " / "
					+ (Timer.getFPGATimestamp() - startTime) + " seconds elapsed");
			StatusCode settingsCode = configurator.apply(canCoderConfig);
			StatusCode sensorCode = thisCancoder.getAbsolutePosition().setUpdateFrequency(20);

			goodInit = settingsCode == StatusCode.OK && sensorCode == StatusCode.OK;

			timedOut = (Timer.getFPGATimestamp()) - startTime >= BOOT_UP_ERROR_ALLOWANCE_TIME;
			attempt++;
		}

		return thisCancoder;
	}

	private Cancoders() {
		mFrontLeft = build(Constants.Ports.FL_CANCODER);
		mFrontLeftObserver = new CanTimeObserver(mFrontLeft);

		mFrontRight = build(Constants.Ports.FR_CANCODER);
		mFrontRightObserver = new CanTimeObserver(mFrontRight);

		mBackLeft = build(Constants.Ports.BL_CANCODER);
		mBackLeftObserver = new CanTimeObserver(mBackLeft);

		mBackRight = build(Constants.Ports.BR_CANCODER);
		mBackRightObserver = new CanTimeObserver(mBackRight);
	}

	public boolean allHaveBeenInitialized() {
		return mFrontLeftObserver.hasUpdate()
				&& mFrontRightObserver.hasUpdate()
				&& mBackLeftObserver.hasUpdate()
				&& mBackRightObserver.hasUpdate();
	}

	public CANcoder getFrontLeft() {
		return mFrontLeft;
	}

	public CANcoder getFrontRight() {
		return mFrontRight;
	}

	public CANcoder getBackLeft() {
		return mBackLeft;
	}

	public CANcoder getBackRight() {
		return mBackRight;
	}
}
