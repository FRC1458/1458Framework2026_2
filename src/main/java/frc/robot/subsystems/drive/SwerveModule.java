package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.drivers.TalonFXManager;
import frc.robot.lib.swerve.SwerveModuleConstants;
import frc.robot.lib.util.Conversions;

public class SwerveModule extends SubsystemBase {
    public final String name;
    private final SwerveModuleConstants mConstants;
	private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private static class PeriodicIO {		// Inputs
		public double timestamp = 0.0;
		public double rotationPosition = 0.0;
		public double rotationVelocity = 0.0;
		public double drivePosition = 0.0;
		public double driveVelocity = 0.0;

        SwerveModuleState currentState = new SwerveModuleState();
        SwerveModuleState targetState = new SwerveModuleState();
        double targetDriveVelocity = 0;
        ControlRequest angleRequest = new NeutralOut();
        ControlRequest driveRequest = new NeutralOut();
    }

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder mCanCoder;
    
    private DCMotorSim mAngleMotorSim;
    private DCMotorSim mDriveMotorSim;


    public SwerveModule(String moduleName, SwerveModuleConstants constants, CANcoder canCoder) {
        name = moduleName;
        setName("Module " + moduleName);
        mConstants = constants;

        mAngleMotor = TalonFXManager.createMotor(name + " Angle Motor", constants.angleMotorID, "Angle");
        mDriveMotor = TalonFXManager.createMotor(name + " Drive Motor", constants.driveMotorID, "Drive");

        mDriveMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Drive.DRIVE_GEAR_RATIO),
				DCMotor.getKrakenX60Foc(1));
		mAngleMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Drive.ANGLE_GEAR_RATIO),
				DCMotor.getFalcon500Foc(1));

        mCanCoder = canCoder;

        mSignals[0] = mDriveMotor.getRotorPosition();
		mSignals[1] = mDriveMotor.getRotorVelocity();
		mSignals[2] = mAngleMotor.getRotorPosition();
		mSignals[3] = mAngleMotor.getRotorVelocity();
        SmartDashboard.putData(this);
    }

    /**
     * Sets the target state of this module.
     * @param state The target {@code SwerveModuleState}.
     */
    public void setTargetState(SwerveModuleState state) {
        mPeriodicIO.targetState = state;
    }

    @Override
    public void periodic() {
        setVelocity(mPeriodicIO.targetState);

        mAngleMotor.setControl(mPeriodicIO.angleRequest);
        mDriveMotor.setControl(mPeriodicIO.driveRequest);

        Rotation2d angleRotation = new Rotation2d(mCanCoder.getPosition().getValue());
        double wheelSpeed = Conversions.RPSToMPS(
                mDriveMotor.getVelocity().getValueAsDouble(),
                Constants.Drive.WHEEL_CIRCUMFERENCE,
                Constants.Drive.DRIVE_GEAR_RATIO);

        mPeriodicIO.currentState = new SwerveModuleState(wheelSpeed, angleRotation);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState mDriveMotorSimState = mDriveMotor.getSimState();
        TalonFXSimState mAngleMotorSimState = mAngleMotor.getSimState();
        CANcoderSimState angleEncoderSimState = mCanCoder.getSimState();
        double batteryVoltage = RobotController.getBatteryVoltage();
        mDriveMotorSimState.setSupplyVoltage(batteryVoltage);
        mAngleMotorSimState.setSupplyVoltage(batteryVoltage);
        angleEncoderSimState.setSupplyVoltage(batteryVoltage);

        mDriveMotorSim.setInputVoltage(mDriveMotorSimState.getMotorVoltageMeasure().in(Volts));
        mDriveMotorSim.update(TimedRobot.kDefaultPeriod);
        mDriveMotorSimState.setRawRotorPosition(
            mDriveMotorSim.getAngularPosition().times(Constants.Drive.DRIVE_GEAR_RATIO));
        mDriveMotorSimState.setRotorVelocity(
            mDriveMotorSim.getAngularVelocity().times(Constants.Drive.DRIVE_GEAR_RATIO));

        mAngleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltageMeasure().in(Volts));
        mAngleMotorSim.update(Constants.DT);
        mAngleMotorSimState.setRawRotorPosition(
            mAngleMotorSim.getAngularPosition().times(Constants.Drive.ANGLE_GEAR_RATIO));
        mAngleMotorSimState.setRotorVelocity(
            mAngleMotorSim.getAngularVelocity().times(Constants.Drive.ANGLE_GEAR_RATIO));

        angleEncoderSimState.setRawPosition(
            mAngleMotorSim.getAngularPosition());
        angleEncoderSimState.setVelocity(
            mAngleMotorSim.getAngularVelocity());
    }

    /**
     * Gets the current state of the module.
     */
    public SwerveModuleState getState() {
        return mPeriodicIO.currentState;
    }

    /**
     * Sets the target velocities of the swerve module.
     * @param targetState The target {@code SwerveModuleState}.
     */
    public void setVelocity(SwerveModuleState targetState) {
        boolean flip = setSteeringAngleOptimized(targetState.angle.unaryMinus());
        mPeriodicIO.targetDriveVelocity = targetState.speedMetersPerSecond * (flip ? -1.0 : 1.0);

        double rotorSpeed = Conversions.MPSToRPS(
                mPeriodicIO.targetDriveVelocity,
                Constants.Drive.WHEEL_CIRCUMFERENCE,
                Constants.Drive.DRIVE_GEAR_RATIO);

        if (Math.abs(rotorSpeed) < 0.01) {
            mPeriodicIO.driveRequest = new NeutralOut();
        } else {
            mPeriodicIO.driveRequest = new VelocityVoltage(rotorSpeed);
        }
    }

    /**
     * Sets the angle of the swerve module, and flips it if it is more efficient to.
     * @param targetAngle The desire angle of the swerve module.
     * @return Whether to flip the drive direction.
     */
    private boolean setSteeringAngleOptimized(Rotation2d targetAngle) {
        boolean flip = false;

        double currentAngleDegrees = mCanCoder.getPosition().getValueAsDouble() * 360.0;
        Rotation2d currentAngle = Rotation2d.fromDegrees(currentAngleDegrees);

        Rotation2d delta = targetAngle.minus(currentAngle);
        double deltaDegrees = delta.getDegrees();

        if (deltaDegrees > 90.0) {
            deltaDegrees -= 180.0;
            flip = true;
        } else if (deltaDegrees < -90.0) {
            deltaDegrees += 180.0;
            flip = true;
        }

        setSteeringAngleRaw(currentAngleDegrees + deltaDegrees);
        return flip;
    }

    /**
     * Sets the target angle position of the swerve module.
     * @param angleDegrees The angle position, in degrees.
     */
    private void setSteeringAngleRaw(double angleDegrees) {
        double rotorPosition = Conversions.degreesToRotation(angleDegrees, Constants.Drive.ANGLE_GEAR_RATIO);
        mPeriodicIO.angleRequest = new PositionVoltage(rotorPosition);
    }

    public synchronized void refreshSignals() {
 		mPeriodicIO.rotationVelocity = mAngleMotor.getRotorVelocity().getValueAsDouble();
		mPeriodicIO.driveVelocity = mDriveMotor.getRotorVelocity().getValueAsDouble();

		mPeriodicIO.rotationPosition = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
				mAngleMotor.getRotorPosition(), mAngleMotor.getRotorVelocity());
		mPeriodicIO.drivePosition = mDriveMotor.getRotorPosition().getValueAsDouble();
	}

    /**
     * Gets the distance the drive wheel has traversed.
     * @return The distance, in meters.
     */
    public double getDriveDistance() {
		return Conversions.rotationsToMeters(
				mPeriodicIO.drivePosition,
				Constants.Drive.WHEEL_CIRCUMFERENCE,
				Constants.Drive.DRIVE_GEAR_RATIO);
	}

    /**
     * Gets the speed of the drive wheel;
     * @return The speed, in meters per second.
     */
    public double getDriveSpeed() {
        return Conversions.RPSToMPS(
            mPeriodicIO.driveVelocity,
            Constants.Drive.WHEEL_CIRCUMFERENCE,
            Constants.Drive.DRIVE_GEAR_RATIO);
    }

    /**
     * Gets the current angle of the swerve module.
     * @return The angle, as a {@code Rotation2d}.
     */
    public Rotation2d getModuleAngle() {
		return Rotation2d.fromDegrees(getModuleAngleDegrees());
	}

    /**
     * Gets the current angle of the swerve module.
     * @return The angle, in degrees.
     */
    public double getModuleAngleDegrees() {
		return Conversions.rotationsToDegrees(mPeriodicIO.rotationPosition, Constants.Drive.ANGLE_GEAR_RATIO);
	}

    public double getModuleAngleSpeed() {
        return Units.RotationsPerSecond.of(mPeriodicIO.rotationVelocity).in(Units.DegreesPerSecond);
    }

    /**
     * Retrieves the status signals.
     * @return The status signals from this swerve module.
     */
    public BaseStatusSignal[] getUsedStatusSignals() {
		return mSignals;
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("/AngleDegrees", this::getModuleAngleDegrees, null);
        builder.addDoubleProperty("/AngleSpeedDegreesPerSecond", this::getModuleAngleSpeed, null);
        builder.addDoubleProperty("/DriveSpeedMetersPerSecond", this::getDriveSpeed, null);
        builder.addDoubleProperty("/TargetAngleDegrees", () -> mPeriodicIO.targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("/TargetDriveSpeedMetersPerSecond", () -> mPeriodicIO.targetState.speedMetersPerSecond, null);
    }
}
