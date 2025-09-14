package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.swerve.COTSTalonFXSwerveConstants;

/**
 * All constants belong here.
 */
public final class Constants {
	public static final double DT = 0.02;
	public static final double DEADBAND = 1e-6;
	public static final double LONG_CANT_TIMEOUT_MS = 0;

	public static final class Controllers {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final double DRIVER_DEADBAND = 0.07;
	}

	public static final class Odometry {
		public static final int OBSERVATION_BUFFER_SIZE = 50;
		public static final Matrix<N2, N1> STATE_STD_DEVS = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
		public static final Matrix<N2, N1> LOCAL_MEASUREMENT_STD_DEVS = VecBuilder.fill(
				Math.pow(0.02, 1), // vision
				Math.pow(0.02, 1));
	}

	public static final class Mover {
		public static final int driverId = 1234;
	}

	public static final class Drive {
		public static final COTSTalonFXSwerveConstants SWERVE_MODULE_TYPE =
			COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        public static final double TRACK_WIDTH = Units.Inches.of(24).in(Units.Meters);
        public static final double WHEEL_BASE =	Units.Inches.of(24).in(Units.Meters);
        public static final double WHEEL_CIRCUMFERENCE = SWERVE_MODULE_TYPE.wheelCircumference;
        public static final double WHEEL_DIAMETER = SWERVE_MODULE_TYPE.wheelDiameter; 

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        };	
		
		public static final double DRIVE_GEAR_RATIO = SWERVE_MODULE_TYPE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = SWERVE_MODULE_TYPE.angleGearRatio;

		public static final double MAX_SPEED = 4.0;
		public static final double MAX_ACCEL = 3.0;
		public static final double MAX_ROTATION_SPEED = 540.0;
		public static final double MAX_ROTATION_ACCEL = 720.0;

		public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int ANGLE_CURRENT_THRESHOLD = 30;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 30;
        public static final int DRIVE_CURRENT_THRESHOLD = 45; 
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static final double DRIVE_MOTOR_KV = 12 * Math.PI * WHEEL_DIAMETER / (DRIVE_GEAR_RATIO * MAX_SPEED);

        public static final PIDFConstants ANGLE_MOTOR_PIDF_CONSTANTS = new PIDFConstants(
            2.0, 0.0, 0.0, 0);
        public static final PIDFConstants DRIVE_MOTOR_PIDF_CONSTANTS = new PIDFConstants(
            1.2, 0.005, 0.0, DRIVE_MOTOR_KV);

		public static final double OPEN_LOOP_RAMP = 0.25;
		public static final double CLOSED_LOOP_RAMP = 0.0;

		public static final boolean INVERT_GYRO = false;

		public static enum ModuleConstants {
			/** Module 0 */
			FRONT_LEFT ( 
				8, 10, 7, Rotation2d.fromRotations(0.142334), true, false),
			/** Module 1 */
			FRONT_RIGHT ( 
				9, 11, 6, Rotation2d.fromRotations(0.427246), true, false),
			/** Module 2 */
			BACK_LEFT ( 
				3, 5, 0, Rotation2d.fromRotations(0.174316), true, false),
			/** Module 3 */
			BACK_RIGHT ( 
				4, 2, 1, Rotation2d.fromRotations(0.413330), false, false);
			
			public final int driveMotorID;
			public final int angleMotorID;
			public final int cancoderID;
			public final Rotation2d angleOffset;
			public final boolean driveInvert;
			public final boolean angleInvert;
		
			/**
			 * Swerve Module Constants to be used when creating swerve modules.
			 * @param driveMotorID
			 * @param angleMotorID
			 * @param canCoderID
			 * @param angleOffset
			 */
			private ModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean driveInvert, boolean angleInvert) {
				this.driveMotorID = driveMotorID;
				this.angleMotorID = angleMotorID;
				this.cancoderID = canCoderID;
				this.angleOffset = angleOffset;
				this.driveInvert = driveInvert;
				this.angleInvert = angleInvert;
			}
		}
	}

	public static final class Auto {
		public static final PIDFConstants TRANSLATION_CONSTANTS = 
			new PIDFConstants(3.5, 0.0, 0.0, 1.0);

		public static final ProfiledPIDFConstants TRANSLATION_CONSTANTS2 = 
			new ProfiledPIDFConstants(3.5, 0.0, 0.0, 1.0, 
				new TrapezoidProfile.Constraints(
					Drive.MAX_SPEED, 
					Drive.MAX_ACCEL));

		public static final ProfiledPIDFConstants ROTATION_CONSTANTS = 
			new ProfiledPIDFConstants(4.0, 0.0, 0.0, 1.0, 
				new TrapezoidProfile.Constraints(
					Drive.MAX_ROTATION_SPEED, 
					Drive.MAX_ROTATION_ACCEL));
		public static final double ACCELERATION_CONSTANT = 0;
	}

	public static final class Limelight { //TODO: this must be tuned to specific robot
		public static enum VisionDeviceConstants {
			L_CONSTANTS (
				"limelight-left",
				new Transform2d(
					new Translation2d(Units.Inches.of(10.5), Units.Inches.of(1.23)),
					Rotation2d.fromDegrees(-90)),
				0, 1600, 1200),
			
			R_CONSTANTS (
				"limelight-right",
				new Transform2d(
                    new Translation2d(Units.Inches.of(10.78), Units.Inches.of(2)),
                    Rotation2d.fromDegrees(90)),
				0, 1600, 1200),

			F_CONSTANTS (
				"limelight-front",
				new Transform2d(
					new Translation2d(Units.Inches.of(11.11), Units.Inches.of(4.28)),
					Rotation2d.fromDegrees(0)),
				0, 1600, 1200),

			B_CONSTANTS (
				"limelight-back",
				new Transform2d(
					new Translation2d(Units.Inches.of(0), Units.Inches.of(-0.96)),
					Rotation2d.fromDegrees(180)),
				0, 1600, 1200);

			public final String tableName;
			public final Transform2d robotToCamera;
			public final int cameraId;
			public final int cameraResolutionWidth;
			public final int cameraResolutionHeight;
			private VisionDeviceConstants(
				String tableName, 
				Transform2d robotToCamera,
				int cameraId, 
				int cameraResolutionWidth,
				int cameraResolutionHeight
			) {
				this.tableName = tableName;
				this.robotToCamera = robotToCamera;
				this.cameraId = cameraId;
				this.cameraResolutionWidth = cameraResolutionWidth;
				this.cameraResolutionHeight = cameraResolutionHeight;
			}
		}
    }

	public static enum Calibration {
		DriveRotation (
			Units.Volts.of(1).per(Units.Second),
			Units.Volts.of(7)),

		DriveTranslation (
			Units.Volts.of(1).per(Units.Second),
			Units.Volts.of(7)),

		AngleMotor (
			Units.Volts.of(1).per(Units.Second),
			Units.Volts.of(7)),

		DriveMotor (
			Units.Volts.of(1).per(Units.Second),
			Units.Volts.of(4));

		public final Velocity<VoltageUnit> RAMP_RATE;
		public final Voltage DYNAMIC_VOLTAGE;
		private Calibration(Velocity<VoltageUnit> rampRate, Voltage dynamicVoltage) {
			RAMP_RATE = rampRate;
			DYNAMIC_VOLTAGE = dynamicVoltage;
		}
	}

	public static enum Port { // TODO: this must be tuned to the specific robot
		FL_CANCODER (7, "CV"),
		FR_CANCODER (6, "CV"),
		BL_CANCODER (14, "CV"),
		BR_CANCODER (1, "CV"),
		DR_CANCODER (1234, "CV"),
		LEDS (21, "CV"),
		PIGEON (60, "CV");

		public final int id;
		public final String bus;
		private Port(int id, String bus) {
			this.id = id;
			this.bus = bus;
		}
	}	
	
	public static final class Pathplanner {
		public static RobotConfig config;
		static {
			try {
				config = RobotConfig.fromGUISettings();
			} catch (Exception e) {
				DriverStation.reportError("Pathplanner configs failed to load ", e.getStackTrace());
			}
		}
		public static final PathConstraints GLOBAL_CONSTRAINTS = 
			new PathConstraints(Drive.MAX_SPEED * 0.85, 
								Drive.MAX_ACCEL * 0.85, 
								Drive.MAX_ROTATION_SPEED * 0.85, 
								Drive.MAX_ROTATION_ACCEL * 0.85);
		public static final double GENERATION_WAIT_TIME = 5;
	}
}
