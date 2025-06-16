package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import frc.robot.Constants;
import frc.robot.Constants.Ports;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Pigeon extends SubsystemBase {
	private static Pigeon mInstance;

	public static Pigeon getInstance() {
		if (mInstance == null) {
			mInstance = new Pigeon(Ports.PIGEON);
		}
		return mInstance;
	}

	private final Pigeon2 mGyro;

	// Configs
	private boolean inverted = Constants.Drive.INVERT_GYRO;
	private Rotation2d yawAdjustmentAngle = new Rotation2d();
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	private double simAngularVelocity = 0.0;

	private Pigeon(int port) {
		mGyro = new Pigeon2(port, "CV");
		mGyro.getConfigurator().apply(new Pigeon2Configuration());
	}

	public Rotation2d getYaw() {
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.unaryMinus());
		if (inverted) {
			return angle.unaryMinus();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.unaryMinus());
	}

	public Rotation2d getPitch() {
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.unaryMinus()).unaryMinus();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
				.rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle =
				getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle =
				getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {		
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValueAsDouble(getYawStatusSignal(), getRateStatusSignal()));
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(mGyro.getRoll().getValueAsDouble());
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(mGyro.getPitch().getValueAsDouble());
	}

	public StatusSignal<Angle> getYawStatusSignal() {
		return mGyro.getYaw();
	}

	public StatusSignal<AngularVelocity> getRateStatusSignal() {
		return mGyro.getAngularVelocityZDevice(); 
	}

	public void setSimAngularVelocity(double angularVelocity) {
		this.simAngularVelocity = angularVelocity;
	}

	@Override
	public void simulationPeriodic() {
		Pigeon2SimState gyroSimState = mGyro.getSimState();

		gyroSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		Rotation2d angleChange = Rotation2d.fromRadians(simAngularVelocity * TimedRobot.kDefaultPeriod);
		Rotation2d angle = getUnadjustedYaw().plus(angleChange);
		gyroSimState.setRawYaw(angle.getDegrees());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Subsystem");
		builder.addDoubleProperty("/RollDegrees", this.getRoll()::getDegrees, null);
		builder.addDoubleProperty("/PitchDegrees", this.getPitch()::getDegrees, null);
		builder.addDoubleProperty("/YawDegrees", this.getYaw()::getDegrees, null);
	}
}
