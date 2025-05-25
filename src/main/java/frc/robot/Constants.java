package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
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

	public static final class Controllers {
		public static final int DRIVER_CONTROLLER_PORT = 0;
	}

	public static final class Drive {
		public static final COTSTalonFXSwerveConstants SWERVE_MODULE_TYPE =
			COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        public static final double TRACK_WIDTH = Units.Inches.of(24).in(Units.Meters);
        public static final double WHEEL_BASE =	Units.Inches.of(24).in(Units.Meters);
        public static final double WHEEL_CIRCUMFERENCE = SWERVE_MODULE_TYPE.wheelCircumference;
        public static final double WHEEL_DIAMETER = SWERVE_MODULE_TYPE.wheelDiameter; 

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        };	
		
		public static final double MAX_SPEED = Units.MetersPerSecond.of(4.0).in(Units.MetersPerSecond);
		public static final double MAX_ACCEL = Units.MetersPerSecondPerSecond.of(3.0).in(Units.MetersPerSecondPerSecond);
		public static final double MAX_ROTATION_SPEED = Units.DegreesPerSecond.of(540.0).in(Units.RadiansPerSecond);
		public static final double MAX_ROTATION_ACCEL = Units.DegreesPerSecondPerSecond.of(720.0).in(Units.RadiansPerSecondPerSecond);
		
		
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
		public static final PIDFConstants TRANSLATION_CONSTANTS = new PIDFConstants(
			5.5,
			0,
			0,
			1
		);
		public static final ProfiledPIDFConstants ROTATION_CONSTANTS = new ProfiledPIDFConstants(
			4,
			0, 
			0, 
			1, 
			new TrapezoidProfile.Constraints(
				Drive.MAX_SPEED, 
				Drive.MAX_ACCEL
			)
		);
	}
}
