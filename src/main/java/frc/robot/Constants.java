package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import org.json.simple.parser.ParseException;

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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.drivers.CanDeviceId;
import frc.robot.lib.swerve.COTSTalonFXSwerveConstants;
import frc.robot.lib.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double DT = 0.02;
	public static final double K_EPSILON = 1e-6;
	public static final double LONG_CANT_TIMEOUT_MS = 0;

	public static final class Controllers {
		public static final int DRIVER_CONTROLLER_PORT = 0;
	}

	public static final class Odometry {
		public static final int OBSERVATION_BUFFER_SIZE = 50;
		public static final Matrix<N2, N1> STATE_STD_DEVS = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
		public static final Matrix<N2, N1> LOCAL_MEASUREMENT_STD_DEVS = VecBuilder.fill(
				Math.pow(0.02, 1), // vision
				Math.pow(0.02, 1));
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

		public static final double MAX_SPEED = Units.MetersPerSecond.of(4.0).in(Units.MetersPerSecond);
		public static final double MAX_ACCEL = Units.MetersPerSecondPerSecond.of(3.0).in(Units.MetersPerSecondPerSecond);
		public static final double MAX_ROTATION_SPEED = Units.DegreesPerSecond.of(540.0).in(Units.RadiansPerSecond);
		public static final double MAX_ROTATION_ACCEL = Units.DegreesPerSecondPerSecond.of(720.0).in(Units.RadiansPerSecondPerSecond);

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

		public static final class Modules {
			/* Module Specific Constants */
			/* Front Left Module - Module 0 */
			public static final class FrontLeft { //TODO: This must be tuned to specific robot
				public static final int DRIVE_MOTOR_ID = 8;
				public static final int ANGLE_MOTOR_ID = 10;
				public static final int CANCODER_ID = 7;
				public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.142334);
				public static final boolean IS_DRIVE_INVERTED = true;
				public static final boolean IS_ANGLE_INVERTED = false;
				public static final SwerveModuleConstants CONSTANTS = 
					new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
			}

			/* Front Right Module - Module 1 */
			public static final class FrontRight { //TODO: This must be tuned to specific robot
				public static final int DRIVE_MOTOR_ID = 9;
				public static final int ANGLE_MOTOR_ID = 11;
				public static final int CANCODER_ID = 6;
				public static final boolean IS_DRIVE_INVERTED = true;
				public static final boolean IS_ANGLE_INVERTED = false;
				public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.427246);
				public static final SwerveModuleConstants CONSTANTS = 
					new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
			}
			
			/* Back Left Module - Module 2 */
			public static final class BackLeft { //TODO: This must be tuned to specific robot
				public static final int DRIVE_MOTOR_ID = 3;
				public static final int ANGLE_MOTOR_ID = 5;
				public static final int CANCODER_ID = 0;
				public static final boolean IS_DRIVE_INVERTED = true;
				public static final boolean IS_ANGLE_INVERTED = false;
				public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.174316);
				public static final SwerveModuleConstants CONSTANTS = 
					new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
			}

			/* Back Right Module - Module 3 */
			public static final class BackRight { //TODO: This must be tuned to specific robot
				public static final int DRIVE_MOTOR_ID = 4;
				public static final int ANGLE_MOTOR_ID = 2;
				public static final int CANCODER_ID = 1;
				public static final boolean IS_DRIVE_INVERTED = false;
				public static final boolean IS_ANGLE_INVERTED = false;
				public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.413330);
				public static final SwerveModuleConstants CONSTANTS =
					new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
			}
		}
	}

	public static final class Auto {
		public static final PIDFConstants TRANSLATION_CONSTANTS = 
			new PIDFConstants(5.5,0,0,1);
		public static final ProfiledPIDFConstants ROTATION_CONSTANTS = 
			new ProfiledPIDFConstants(4,0, 0, 1, 
				new TrapezoidProfile.Constraints(
					Drive.MAX_ROTATION_SPEED, 
					Drive.MAX_ROTATION_ACCEL
				)
			);
	}

	public static final class Limelight { //TODO: this must be tuned to specific robot
		public static final class VisionDeviceConstants {
			public String kTableName = "limelight-front";//"limelight-c";
			public Transform2d kRobotToCamera = new Transform2d();
			public int kCameraId = 0;
			public int kCameraResolutionWidth = 1600;
			public int kCameraResolutionHeight = 1200;
		}


        public static VisionDeviceConstants L_CONSTANTS = new VisionDeviceConstants();
        public static VisionDeviceConstants R_CONSTANTS = new VisionDeviceConstants();
        public static VisionDeviceConstants F_CONSTANTS = new VisionDeviceConstants();
        public static VisionDeviceConstants B_CONSTANTS = new VisionDeviceConstants();

        static {
            L_CONSTANTS.kTableName = "limelight-left";    
            L_CONSTANTS.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.Inches.of(10.5), Units.Inches.of(1.23)),
                    Rotation2d.fromDegrees(-90));

            R_CONSTANTS.kTableName = "limelight-right";  
            R_CONSTANTS.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.Inches.of(10.78), Units.Inches.of(2)),
                    Rotation2d.fromDegrees(90));
            
            F_CONSTANTS.kTableName = "limelight-front";
            F_CONSTANTS.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.Inches.of(11.11), Units.Inches.of(4.28)),
                    Rotation2d.fromDegrees(0));

            B_CONSTANTS.kTableName = "limelight-back";
            B_CONSTANTS.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.Inches.of(0), Units.Inches.of(-0.96)),
                    Rotation2d.fromDegrees(180));
        }

    }

	public static final class Ports {
		public static final CanDeviceId FL_CANCODER = new CanDeviceId(7, "CV");
	
		public static final CanDeviceId FR_CANCODER = new CanDeviceId(6, "CV");
	
		public static final CanDeviceId BL_CANCODER = new CanDeviceId(14, "CV");
	
		public static final CanDeviceId BR_CANCODER = new CanDeviceId(1, "CV");
	
		public static final int PIGEON = 60; // TODO: this must be tuned to the specific robot
		
		public static final CanDeviceId LEDS = new CanDeviceId(21, "CV");
	}	
	
	public static final class Pathplanner {
		public static RobotConfig config;
		static {
			try {
				config = RobotConfig.fromGUISettings();
			} catch (Exception e) {
				DriverStation.reportError("Pathplanner configs failed to load", e.getStackTrace());
			}
		}
		public static final PathConstraints GLOBAL_CONSTRAINTS = 
			new PathConstraints(Drive.MAX_SPEED, 
								Drive.MAX_ACCEL, 
								Drive.MAX_ROTATION_SPEED, 
								Drive.MAX_ROTATION_ACCEL);
	}
}
