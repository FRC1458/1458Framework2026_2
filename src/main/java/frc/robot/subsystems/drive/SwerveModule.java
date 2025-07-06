package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.drivers.TalonFXManager;
import frc.robot.Constants;
import frc.robot.lib.util.Conversions;

public class SwerveModule extends SubsystemBase {
    public final String name;
	private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

    private SwerveModuleIO io = new SwerveModuleIO();
    public static class SwerveModuleIO {
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

    private State state = State.DEFAULT;
    public static enum State {
        DEFAULT,
        ANGLE_TUNING,
        DRIVE_TUNING
    }

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder cancoder;
    
    private DCMotorSim angleMotorSim;
    private DCMotorSim driveMotorSim;

    public SwerveModule(Constants.Drive.Modules constants, CANcoder cancoder) {
        name = constants.name();
        setName("Module " + name);

        angleMotor = TalonFXManager.createMotor(name + " Angle Motor", constants.angleMotorID, "Angle");
        driveMotor = TalonFXManager.createMotor(name + " Drive Motor", constants.driveMotorID, "Drive");

        driveMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Drive.DRIVE_GEAR_RATIO),
				DCMotor.getKrakenX60Foc(1));
		angleMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Drive.ANGLE_GEAR_RATIO),
				DCMotor.getFalcon500Foc(1));

        this.cancoder = cancoder;

        mSignals[0] = driveMotor.getRotorPosition();
		mSignals[1] = driveMotor.getRotorVelocity();
		mSignals[2] = angleMotor.getRotorPosition();
		mSignals[3] = angleMotor.getRotorVelocity();
        SmartDashboard.putData(this);
    }

    /**
     * Sets the target state of this module.
     * @param state The target {@code SwerveModuleState}.
     */
    public void setTargetState(SwerveModuleState state) {
        io.targetState = state;
    }

    @Override
    public void periodic() {
        switch (state) {
            case DEFAULT:
                setVelocity(io.targetState);
                break;
            case ANGLE_TUNING:
                break;
            case DRIVE_TUNING:
                break;
            default:
                setVelocity(io.targetState);
                break;
        }

        angleMotor.setControl(io.angleRequest);
        driveMotor.setControl(io.driveRequest);

        Rotation2d angleRotation = new Rotation2d(cancoder.getPosition().getValue());
        double wheelSpeed = Conversions.RPSToMPS(
                driveMotor.getVelocity().getValueAsDouble(),
                Constants.Drive.WHEEL_CIRCUMFERENCE,
                Constants.Drive.DRIVE_GEAR_RATIO);

        io.currentState = new SwerveModuleState(wheelSpeed, angleRotation);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState mDriveMotorSimState = driveMotor.getSimState();
        TalonFXSimState mAngleMotorSimState = angleMotor.getSimState();
        CANcoderSimState angleEncoderSimState = cancoder.getSimState();
        double batteryVoltage = RobotController.getBatteryVoltage();
        mDriveMotorSimState.setSupplyVoltage(batteryVoltage);
        mAngleMotorSimState.setSupplyVoltage(batteryVoltage);
        angleEncoderSimState.setSupplyVoltage(batteryVoltage);

        driveMotorSim.setInputVoltage(mDriveMotorSimState.getMotorVoltageMeasure().in(Units.Volts));
        driveMotorSim.update(TimedRobot.kDefaultPeriod);
        mDriveMotorSimState.setRawRotorPosition(
            driveMotorSim.getAngularPosition().times(Constants.Drive.DRIVE_GEAR_RATIO));
        mDriveMotorSimState.setRotorVelocity(
            driveMotorSim.getAngularVelocity().times(Constants.Drive.DRIVE_GEAR_RATIO));

        angleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltageMeasure().in(Units.Volts));
        angleMotorSim.update(Constants.DT);
        mAngleMotorSimState.setRawRotorPosition(
            angleMotorSim.getAngularPosition().times(Constants.Drive.ANGLE_GEAR_RATIO));
        mAngleMotorSimState.setRotorVelocity(
            angleMotorSim.getAngularVelocity().times(Constants.Drive.ANGLE_GEAR_RATIO));

        angleEncoderSimState.setRawPosition(
            angleMotorSim.getAngularPosition());
        angleEncoderSimState.setVelocity(
            angleMotorSim.getAngularVelocity());
    }

    /**
     * Gets the current state of the module.
     */
    public SwerveModuleState getState() {
        return io.currentState;
    }

    /**
     * Sets the target velocities of the swerve module.
     * @param targetState The target {@code SwerveModuleState}.
     */
    public void setVelocity(SwerveModuleState targetState) {
        boolean flip = setSteeringAngleOptimized(targetState.angle.unaryMinus());
        io.targetDriveVelocity = targetState.speedMetersPerSecond * (flip ? -1.0 : 1.0);

        double rotorSpeed = Conversions.MPSToRPS(
                io.targetDriveVelocity,
                Constants.Drive.WHEEL_CIRCUMFERENCE,
                Constants.Drive.DRIVE_GEAR_RATIO);

        if (Math.abs(rotorSpeed) < 0.01) {
            io.driveRequest = new NeutralOut();
        } else {
            io.driveRequest = new VelocityVoltage(rotorSpeed);
        }
    }

    /**
     * Sets the angle of the swerve module, and flips it if it is more efficient to.
     * @param targetAngle The desire angle of the swerve module.
     * @return Whether to flip the drive direction.
     */
    private boolean setSteeringAngleOptimized(Rotation2d targetAngle) {
        boolean flip = false;

        double currentAngleDegrees = cancoder.getPosition().getValueAsDouble() * 360.0;
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
        io.angleRequest = new PositionVoltage(rotorPosition);
    }

    public synchronized void refreshSignals() {
 		io.rotationVelocity = angleMotor.getRotorVelocity().getValueAsDouble();
		io.driveVelocity = driveMotor.getRotorVelocity().getValueAsDouble();

		io.rotationPosition = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            angleMotor.getRotorPosition(), angleMotor.getRotorVelocity());
		io.drivePosition = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            driveMotor.getRotorPosition(), driveMotor.getRotorVelocity());
	}

    /**
     * Gets the distance the drive wheel has traversed.
     * @return The distance, in meters.
     */
    public double getDriveDistance() {
		return Conversions.rotationsToMeters(
            io.drivePosition,
            Constants.Drive.WHEEL_CIRCUMFERENCE,
            Constants.Drive.DRIVE_GEAR_RATIO);
	}

    /**
     * Gets the speed of the drive wheel;
     * @return The speed, in meters per second.
     */
    public double getDriveSpeed() {
        return Conversions.RPSToMPS(
            io.driveVelocity,
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
		return Conversions.rotationsToDegrees(io.rotationPosition, Constants.Drive.ANGLE_GEAR_RATIO);
	}

    /**
     * Gets the speed of the angle.
     * @return The speed, in degrees per second.
     */
    public double getModuleAngleSpeed() {
        return Units.RotationsPerSecond.of(io.rotationVelocity).in(Units.DegreesPerSecond);
    }

    /**
     * Retrieves the status signals.
     * @return The status signals from this swerve module.
     */
    public BaseStatusSignal[] getUsedStatusSignals() {
		return mSignals;
	}
    
    /**
     * The system identification routine for the angle motor.
     * @return The angle motor routine.
     */
    public SysIdRoutine angleRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Constants.Tuning.AngleMotor.RAMP_RATE,
                Constants.Tuning.AngleMotor.DYNAMIC_VOLTAGE,
                null,
                recordState -> SignalLogger.writeString("/Sysid/Angle/State", recordState.toString())
            ), 
            new SysIdRoutine.Mechanism(
                volts -> io.angleRequest = new VoltageOut(volts), null, this));
    }

    /**
     * The system identification routine for the drive motor.
     * @return The drive motor routine.
     */
    public SysIdRoutine driveRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Constants.Tuning.DriveMotor.RAMP_RATE,
                Constants.Tuning.DriveMotor.DYNAMIC_VOLTAGE,
                null,
                recordState -> SignalLogger.writeString("/Sysid/Drive/State", recordState.toString())
            ), 
            new SysIdRoutine.Mechanism(
                volts -> io.driveRequest = new VoltageOut(volts), null, this));
    }

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
	public Command sysIdDynamic(State type, SysIdRoutine.Direction direction) {
		return switch (type) {
            case ANGLE_TUNING ->
                Commands.runOnce(() -> state = type)
                    .andThen(angleRoutine().dynamic(direction))
                    .andThen(Commands.runOnce(() -> state = State.DEFAULT));
            case DRIVE_TUNING ->
                Commands.runOnce(() -> state = type)
                    .andThen(driveRoutine().dynamic(direction))
                    .andThen(Commands.runOnce(() -> state = State.DEFAULT));
            default -> {
                DriverStation.reportWarning("Did not provide a valid tuning type", false);
                yield Commands.none();
            }
        };
	}

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
	public Command sysIdQuasistatic(State type, SysIdRoutine.Direction direction) {
		return switch (type) {
            case ANGLE_TUNING ->
                Commands.runOnce(() -> state = type)
                    .andThen(Commands.runOnce(
                        () -> io.driveRequest = new NeutralOut()))
                    .andThen(angleRoutine().quasistatic(direction))
                    .andThen(Commands.runOnce(() -> state = State.DEFAULT));
            case DRIVE_TUNING ->
                Commands.runOnce(() -> state = type)
                    .andThen(Commands.runOnce(
                        () -> io.angleRequest = new NeutralOut()))
                    .andThen(driveRoutine().quasistatic(direction))
                    .andThen(Commands.runOnce(() -> state = State.DEFAULT));
            default -> {
                DriverStation.reportWarning("Did not provide a valid tuning type", false);
                yield Commands.none();
            }
        };
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("/State", () -> state.name(), null);
        builder.addDoubleProperty("/AngleDegrees", () -> getModuleAngleDegrees(), null);
        builder.addDoubleProperty("/AngleSpeedDegreesPerSecond", () -> getModuleAngleSpeed(), null);
        builder.addDoubleProperty("/DriveSpeedMetersPerSecond", () -> getDriveSpeed(), null);
        builder.addDoubleProperty("/TargetAngleDegrees", () -> io.targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("/TargetDriveSpeedMetersPerSecond", () -> io.targetState.speedMetersPerSecond, null);
    }
}
