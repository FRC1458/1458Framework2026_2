package frc.robot.subsystems;

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
	private static Cancoders mInstance;
	public static Cancoders getInstance() {
		if (mInstance == null) {
			mInstance = new Cancoders();
		}
		return mInstance;
	}

	private final CANcoder mFrontLeft;
	private final CANcoder mFrontRight;
	private final CANcoder mBackLeft;
	private final CANcoder mBackRight;

	private final CanTimestampObserver mFrontRightObserver;
	private final CanTimestampObserver mFrontLeftObserver;
	private final CanTimestampObserver mBackLeftObserver;
	private final CanTimestampObserver mBackRightObserver;

	private static final double kBootUpErrorAllowanceTime = 10.0;

	private static class CanTimestampObserver {
		private final CANcoder cancoder;
		private Optional<Double> lastTimestamp = Optional.empty();
		private int validUpdates = 0;
		private static final int kRequiredValidTimestamps = 10;

		public CanTimestampObserver(CANcoder cancoder) {
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

	private CANcoder build(CanDeviceId canDeviceId) {
		CANcoder thisCancoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());
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
			System.out.println("Initing CANCoder " + canDeviceId.getDeviceNumber() + " / attempt: " + attempt + " / "
					+ (Timer.getFPGATimestamp() - startTime) + " seconds elapsed");
			StatusCode settingsCode = configurator.apply(canCoderConfig);
			StatusCode sensorCode = thisCancoder.getAbsolutePosition().setUpdateFrequency(20);

			goodInit = settingsCode == StatusCode.OK && sensorCode == StatusCode.OK;

			timedOut = (Timer.getFPGATimestamp()) - startTime >= kBootUpErrorAllowanceTime;
			attempt++;
		}

		return thisCancoder;
	}

	private Cancoders() {
		mFrontLeft = build(Ports.FL_CANCODER);
		mFrontLeftObserver = new CanTimestampObserver(mFrontLeft);

		mFrontRight = build(Ports.FR_CANCODER);
		mFrontRightObserver = new CanTimestampObserver(mFrontRight);

		mBackLeft = build(Ports.BL_CANCODER);
		mBackLeftObserver = new CanTimestampObserver(mBackLeft);

		mBackRight = build(Ports.BR_CANCODER);
		mBackRightObserver = new CanTimestampObserver(mBackRight);
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
