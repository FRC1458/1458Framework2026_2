package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class WheelTracker {
	private final SwerveModule[] modules;
	private final Pigeon pigeon;

	private WheelProperties[] wheelProperties = new WheelProperties[4];
	private Pose2d robotPose = new Pose2d(10,10,new Rotation2d(0));
	private Translation2d robotVelocity = new Translation2d(0, 0);
	private BaseStatusSignal[] allSignals;

	private double robotHeading;

	private double timestamp;

	private OdometryThread odometryThread;

	private Field2d robotField = new Field2d();

	public WheelTracker(SwerveModule[] modules) {
		if (modules.length != 4) {
			throw new IllegalArgumentException("Odometry needs 4 modules to run");
		}

		this.modules = modules;
		pigeon = Pigeon.getInstance();

		for (int i = 0; i < wheelProperties.length; i++) {
			WheelProperties w = new WheelProperties();
			Translation2d robotToWheel = new Translation2d(
					Constants.Drive.MODULE_LOCATIONS[i].getX(),
					Constants.Drive.MODULE_LOCATIONS[i].getY());
			w.startingPosition = robotToWheel;
			wheelProperties[i] = w;
		}

		resetModulePoses();

		allSignals = new BaseStatusSignal[(4 * 4) + 2];
		for (int i = 0; i < 4; ++i) {
			var signals = modules[i].getUsedStatusSignals();
			allSignals[(i * 4) + 0] = signals[0];
			allSignals[(i * 4) + 1] = signals[1];
			allSignals[(i * 4) + 2] = signals[2];
			allSignals[(i * 4) + 3] = signals[3];
		}
		allSignals[allSignals.length - 2] = pigeon.getYawStatusSignal();
		allSignals[allSignals.length - 1] = pigeon.getRateStatusSignal();

		for (BaseStatusSignal sig : allSignals) {
			sig.setUpdateFrequency(50);
		}
		odometryThread = new OdometryThread();
		odometryThread.setDaemon(true);
		odometryThread.start();

		robotField.setRobotPose(robotPose);

		SmartDashboard.putData("WheelTracker", robotField);
	}
	private class OdometryThread extends Thread {
		@Override
		public void run() {
			while (true) {
				try {
					BaseStatusSignal.waitForAll(1.0, allSignals);

					for (SwerveModule m : modules) {
						m.refreshSignals(); // No downside to refreshing io reads from multiple threads
					}

					robotHeading = pigeon.getYaw().getRadians();
					updateRobotPose(Timer.getFPGATimestamp());
				} catch (Exception e) {
					e.printStackTrace();
					System.out.println("Failed, see above error");
				}
			}
		}
	}

	private Pose2d last_velocity_sample = new Pose2d();
	private double last_sample_timestamp = 0.0;

	private void updateRobotPose(double timestamp) {
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = Rotation2d.fromRadians(robotHeading);

		double avg_delta = 0.0;
		double[] deltas = new double[4];
		for (int i = 0; i < modules.length; i++) {
			SwerveModule m = modules[i];
			WheelProperties w = wheelProperties[i];
			updateWheelOdometry(m, w);
			double delta = w.estimatedRobotPose
					.getTranslation()
					.plus(robotPose.getTranslation().unaryMinus())
					.getNorm();
			deltas[i] = delta;
			avg_delta += delta;
		}
		avg_delta /= 4;

		int min__dev_idx = 0;
		double min_dev = Double.MAX_VALUE;
		List<WheelProperties> accurateModules = new ArrayList<>();
		for (int i = 0; i < deltas.length; i++) {
			WheelProperties w = wheelProperties[i];
			double dev = Math.abs(deltas[i] - avg_delta);
			if (dev < min_dev) {
				min_dev = dev;
				min__dev_idx = i;
			}
			if (dev <= 0.01) {
				accurateModules.add(w);
			}
		}

		if (accurateModules.isEmpty()) {
			accurateModules.add(wheelProperties[min__dev_idx]);
		}

		int n = accurateModules.size();

		for (WheelProperties w : accurateModules) {
			x += w.estimatedRobotPose.getTranslation().getX();
			y += w.estimatedRobotPose.getTranslation().getY();
		}
		final Pose2d new_pose = new Pose2d(new Translation2d(x / n, y / n), heading);

		// Velocity calcs
		double sample_window = timestamp - last_sample_timestamp;
		if (sample_window > 0.02) {
			final Translation2d translation = (new_pose.transformBy(new Transform2d(
					new Translation2d(-last_velocity_sample.getTranslation().getX(),
							-last_velocity_sample.getTranslation().getY()),
					last_velocity_sample.getRotation().unaryMinus())).getTranslation());
			robotVelocity = new Translation2d(
				translation.getX() * (1.0 / sample_window),
				translation.getY() * (1.0  / sample_window)
			);
			last_sample_timestamp = timestamp;
			last_velocity_sample = new_pose;
		}

		robotPose = new_pose;

		robotField.setRobotPose(new_pose);

		resetModulePoses(robotPose);
	}


	private void updateWheelOdometry(SwerveModule module, WheelProperties props) {
		double currentEncDistance = module.getDriveDistance();
		double deltaEncDistance = currentEncDistance - props.previousEncDistance;
		Rotation2d wheelAngle = module.getModuleAngle().rotateBy(Rotation2d.fromRadians(-robotHeading));
		Translation2d deltaPosition = new Translation2d(
			wheelAngle.getCos() * deltaEncDistance,
			-wheelAngle.getSin() * deltaEncDistance); 
		double xCorrectionFactor = 1.0;
		double yCorrectionFactor = 1.0;

		if (Math.signum(deltaPosition.getX()) == 1.0) {
			xCorrectionFactor = (8.6 / 9.173);

		} else if (Math.signum(deltaPosition.getX()) == -1.0) {
			xCorrectionFactor = (8.27 / 9.173);
		}

		if (Math.signum(deltaPosition.getY()) == 1.0) {
			yCorrectionFactor = (3.638 / 4.0);
		} else if (Math.signum(deltaPosition.getY()) == -1.0) {
			yCorrectionFactor = (3.660 / 4.0);
		}

		deltaPosition = new Translation2d(deltaPosition.getX() * xCorrectionFactor, deltaPosition.getY() * yCorrectionFactor);
		Translation2d updatedPosition = props.position.plus(deltaPosition);
		Pose2d wheelPose = new Pose2d(updatedPosition, Rotation2d.fromRadians(robotHeading));
		props.estimatedRobotPose = new Pose2d(wheelPose.getTranslation().plus(props.startingPosition.unaryMinus()), wheelPose.getRotation());

		props.position = updatedPosition;
		props.previousEncDistance = currentEncDistance;
	}

	public void resetModulePoses(Pose2d mRobotPose) {
		for (int i = 0; i < modules.length; i++) {
			WheelProperties props = wheelProperties[i];
			Translation2d modulePosition = new Pose2d(mRobotPose.getTranslation().plus(props.startingPosition),mRobotPose.getRotation())
					.getTranslation();
			props.position = modulePosition;
		}
	}

	private void resetModulePoses() {
		for (int i = 0; i < modules.length; i++) {
			WheelProperties props = wheelProperties[i];
			props.position = props.startingPosition;
		}
	}


	public synchronized void resetPose(Pose2d pose) {
		robotPose = new Pose2d(pose.getTranslation(), pose.getRotation());
		resetModulePoses(robotPose);
	}

	public class WheelProperties {
		private double previousEncDistance = 0;
		private Translation2d position;
		private Translation2d startingPosition;
		private Pose2d estimatedRobotPose = new Pose2d();
	}

	public synchronized Pose2d getRobotPose() {
		return robotPose;
	}

	public synchronized Translation2d getMeasuredVelocity() { 
		return robotVelocity;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public double wheel0_x() {
		return wheelProperties[0].position.getX();
	}

	public double wheel0_y() {
		return wheelProperties[0].position.getY();
	}

	public double wheel1_x() {
		return wheelProperties[1].position.getX();
	}

	public double wheel1_y() {
		return wheelProperties[1].position.getY();
	}

	public double wheel2_x() {
		return wheelProperties[2].position.getX();
	}

	public double wheel2_y() {
		return wheelProperties[2].position.getY();
	}

	public double wheel3_x() {
		return wheelProperties[3].position.getX();
	}

	public double wheel3_y() {
		return wheelProperties[3].position.getY();
	}

	public double robot_x() {
		return robotPose.getTranslation().getX();
	}

	public double robot_y() {
		return robotPose.getTranslation().getY();
	}
}
