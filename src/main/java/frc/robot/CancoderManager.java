package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class CancoderManager {
	public static CancoderManager cancodersInstance;
	public static CancoderManager getInstance() {
		if (cancodersInstance == null) {
			cancodersInstance = new CancoderManager();
		}
		return cancodersInstance;
	}

	private final CANcoder fl;
	private final CANcoder fr;
	private final CANcoder bl;
	private final CANcoder br;

	private final CanTimeObserver frObs;
	private final CanTimeObserver flObs;
	private final CanTimeObserver blObs;
	private final CanTimeObserver brObs;

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

	private CANcoder build(Constants.Port canDeviceId) {
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

	public CancoderManager() {
		fl = build(Constants.Port.FL_CANCODER);
		flObs = new CanTimeObserver(fl);

		fr = build(Constants.Port.FR_CANCODER);
		frObs = new CanTimeObserver(fr);

		bl = build(Constants.Port.BL_CANCODER);
		blObs = new CanTimeObserver(bl);

		br = build(Constants.Port.BR_CANCODER);
		brObs = new CanTimeObserver(br);
	}

	public boolean allHaveBeenInitialized() {
		return flObs.hasUpdate()
				&& frObs.hasUpdate()
				&& blObs.hasUpdate()
				&& brObs.hasUpdate();
	}

	public CANcoder getFrontLeft() {
		return fl;
	}

	public CANcoder getFrontRight() {
		return fr;
	}

	public CANcoder getBackLeft() {
		return bl;
	}

	public CANcoder getBackRight() {
		return br;
	}
}
