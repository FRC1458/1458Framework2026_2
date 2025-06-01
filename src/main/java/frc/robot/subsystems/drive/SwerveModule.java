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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.drivers.TalonFXManager;
import frc.robot.lib.swerve.SwerveModuleConstants;
import frc.robot.lib.util.Conversions;

public class SwerveModule extends SubsystemBase {
    private final String name;
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
    }

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

        SmartDashboard.putNumber(name + "/AngleDeg", angleRotation.getDegrees());
        SmartDashboard.putNumber(name + "/WheelSpeedMPS", wheelSpeed);
        SmartDashboard.putNumber(name + "/TargetAngleDeg", mPeriodicIO.targetState.angle.getDegrees());
        SmartDashboard.putNumber(name + "/TargetSpeedMPS", mPeriodicIO.targetState.speedMetersPerSecond);
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
            mAngleMotorSim.getAngularPosition());//.times(Constants.Drive.ANGLE_GEAR_RATIO));
        angleEncoderSimState.setVelocity(
            mAngleMotorSim.getAngularVelocity());//.times(Constants.Drive.ANGLE_GEAR_RATIO));
    }


    public SwerveModuleState getState() {
        return mPeriodicIO.currentState;
    }

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

    private boolean setSteeringAngleOptimized(Rotation2d desiredAngle) {
        boolean flip = false;

        double currentAngleDegrees = mCanCoder.getPosition().getValueAsDouble() * 360.0;
        Rotation2d currentAngle = Rotation2d.fromDegrees(currentAngleDegrees);

        Rotation2d delta = desiredAngle.minus(currentAngle);
        double deltaDegrees = delta.getDegrees();

        if (deltaDegrees > 90.0) {
            deltaDegrees -= 180.0;
            flip = true;
        } else if (deltaDegrees < -90.0) {
            deltaDegrees += 180.0;
            flip = true;
        }

        double targetAngle = currentAngleDegrees + deltaDegrees;
        setSteeringAngleRaw(targetAngle);
        return flip;
    }

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

    public double getDriveDistanceMeters() {
		return Conversions.rotationsToMeters(
				mPeriodicIO.drivePosition,
				Constants.Drive.WHEEL_CIRCUMFERENCE,
				Constants.Drive.DRIVE_GEAR_RATIO);
	}

    public BaseStatusSignal[] getUsedStatusSignals() {
		return mSignals;
	}

    public Rotation2d getModuleAngle() {
		return Rotation2d.fromDegrees(getCurrentUnboundedDegrees());
	}

    public double getCurrentUnboundedDegrees() {
		return Conversions.rotationsToDegrees(mPeriodicIO.rotationPosition, Constants.Drive.ANGLE_GEAR_RATIO);
	}
}
