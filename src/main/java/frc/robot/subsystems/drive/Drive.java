package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.ctre.CtreDriveConstants;
import frc.robot.subsystems.drive.ctre.CtreDrive;
import frc.robot.subsystems.drive.ctre.CtreDriveTelemetry;

import java.util.function.UnaryOperator;

public class Drive extends SubsystemBase {
	private static Drive drive2Instance;
    public static Drive getInstance() {
        if (drive2Instance == null) {
            drive2Instance = new Drive();
        }
        return drive2Instance;
    }

	private SwerveDriveState lastReadState;
	public SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final CtreDrive drivetrain = CtreDriveConstants.createDrivetrain();
	private final CtreDriveTelemetry telemetry = new CtreDriveTelemetry(Constants.Drive.MAX_SPEED);
    // Citrus what are you doing
	private Time lastPoseResetTime = BaseUnits.TimeUnit.of(0.0);

	private final Field2d elasticPose = new Field2d();

	private Drive() {
		lastReadState = drivetrain.getState();
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			return driveRequest;
		}));

		if (!Robot.isReal()) {
			drivetrain.resetPose((new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0))));
		}

		drivetrain.getOdometryThread().setThreadPriority(31);
        TelemetryManager.getInstance().addStructPublisher("Mechanisms/Drive", Pose3d.struct, () -> new Pose3d(getPose()));
        TelemetryManager.getInstance().addSendable(this);
	}

	public CtreDrive getCtreDrive() {
		return drivetrain;
	}

	@Override
	public void periodic() {
		lastReadState = drivetrain.getState();
		outputTelemetry();
	}

	public void outputTelemetry() {
		telemetry.telemeterize(lastReadState);
		elasticPose.setRobotPose(getPose());
		SmartDashboard.putData("Elastic Field 2D", elasticPose);
	}

	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

	public Pose2d getPose() {
		return lastReadState.Pose;
	}

	// public void setSwerveRequest(SwerveRequest request) {
	// 	driveRequest = request;
	// }

    public SwerveRequest getSwerveRequest() {
		return driveRequest; 
	}


	// public Command followSwerveRequestCommand(
	// 		SwerveRequest.FieldCentric request, UnaryOperator<SwerveRequest.FieldCentric> updater) {
	// 	return run(() -> setSwerveRequest(updater.apply(request)))
    //         .handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()));
	// }

    public Command teleopCommand() {
        return run(() -> {
            System.out.println("teleopCommand.run() vx=" + Robot.controller.getLeftX() + ", vy = " + Robot.controller.getLeftY() + ", vw=" + Robot.controller.getRightX());
            // setSwerveRequest(new SwerveRequest.FieldCentric()
            //     .withVelocityX(Robot.controller.getLeftX()* Constants.Drive.MAX_SPEED)
            //     .withVelocityY(Robot.controller.getLeftY()* Constants.Drive.MAX_SPEED)
            //     .withRotationalRate(Robot.controller.getRightX()* Constants.Drive.MAX_ROTATION_SPEED));
            // });
//            setSwerveRequest(new SwerveRequest.FieldCentric()
            driveRequest
                .withVelocityX(
                    Robot.controller.getLeftY() * Constants.Drive.MAX_SPEED)
                .withVelocityY(
                    Robot.controller.getLeftX() * Constants.Drive.MAX_SPEED)
                .withRotationalRate(
                    Robot.controller.getRightX() * Constants.Drive.MAX_ROTATION_SPEED);
        });
    }

	public void addVisionUpdate(Pose2d pose, Time timestamp) {
		getCtreDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds));
	}

	public void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		getCtreDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

	public void resetPose(Pose2d pose) {
		getCtreDrive().resetPose(pose);
		lastPoseResetTime =
            Units.Seconds.of(Utils.getCurrentTimeSeconds()).plus(Constants.Drive.POSE_RESET_PREVENTION_TIME);
	}

	public Command resetPoseCommand(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

	public boolean isPitchStable() {
		return drivetrain.getPigeon2().getAngularVelocityYDevice().getValue().abs(Units.DegreesPerSecond)
                < Constants.Drive.MAX_VELOCITY_STABLE
            && drivetrain.getPigeon2().getPitch().getValue().abs(BaseUnits.AngleUnit)
                < Constants.Drive.MAX_PITCH_STABLE;
	}

	public boolean isRollStable() {
		return drivetrain.getPigeon2().getAngularVelocityXDevice().getValue().abs(Units.DegreesPerSecond)
                < Constants.Drive.MAX_VELOCITY_STABLE
            && drivetrain.getPigeon2().getRoll().getValue().abs(BaseUnits.AngleUnit)
                < Constants.Drive.MAX_PITCH_STABLE;
	}

	public boolean isStable() {
		ChassisSpeeds speeds = getState().Speeds;
		return isPitchStable()
            && isRollStable()
            && Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
                .lte(Constants.Drive.kScoringTranslationMaxSpeed)
            && Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(Constants.Drive.kScoringRotationMaxSpeed);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
		builder.addDoubleProperty(
            "Pitch Velocity Degrees Per Second",
            () -> drivetrain
                .getPigeon2()
                .getAngularVelocityYDevice()
                .getValue()
                .in(Units.DegreesPerSecond),
            null);
		builder.addDoubleProperty(
            "Pitch Degrees",
            () -> drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees),
            null);

		builder.addDoubleProperty(
            "Roll Velocity Degrees Per Second",
            () -> drivetrain
                .getPigeon2()
                .getAngularVelocityXDevice()
                .getValue()
                .in(Units.DegreesPerSecond),
            null);
		builder.addDoubleProperty(
            "Roll Degrees",
            () -> drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees),
            null);

		addModuleToBuilder(builder, 0);
		addModuleToBuilder(builder, 1);
		addModuleToBuilder(builder, 2);
		addModuleToBuilder(builder, 3);
	}

	private void addModuleToBuilder(SendableBuilder builder, int module) {
		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Drive/Volts",
            () -> drivetrain
                .getModules()[module]
                .getDriveMotor()
                .getMotorVoltage()
                .getValue()
                .in(Units.Volts),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Rotation/Volts",
            () -> drivetrain
                .getModules()[module]
                .getSteerMotor()
                .getMotorVoltage()
                .getValue()
                .in(Units.Volts),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Drive/Stator Current",
            () -> drivetrain
                .getModules()[module]
                .getDriveMotor()
                .getStatorCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Drive/Temperature Celsius",
            () -> drivetrain
                .getModules()[module]
                .getDriveMotor()
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Rotation/Stator Current",
            () -> drivetrain
                .getModules()[module]
                .getSteerMotor()
                .getStatorCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Drive/Supply Current",
            () -> drivetrain
                .getModules()[module]
                .getDriveMotor()
                .getSupplyCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Rotation/Supply Current",
            () -> drivetrain
                .getModules()[module]
                .getSteerMotor()
                .getSupplyCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "ModuleStates/" + module + "/Rotation/Temperature Celsius",
            () -> drivetrain
                .getModules()[module]
                .getSteerMotor()
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);
	}
}