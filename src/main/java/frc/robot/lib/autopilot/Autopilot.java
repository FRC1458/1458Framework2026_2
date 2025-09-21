package frc.robot.lib.autopilot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Autopilot is a class that tries to drive a target to a goal in 2-D space.
 *
 * Autopilot is a stateless algorithm; as such, it does not "think ahead" and cannot avoid obstacles.
 * Any math that Autopilot needs is already worked out such that only a small amount of computation is necessary on the fly. 
 * Autopilot is designed to be used in a drivetrain's control loop, where the current state of the robot is passed in, and the next velocity is returned.
 * 
 */
public class Autopilot {
	private APProfile m_profile;

	private final double dt = 0.020;

	/**
	 * A class that holds constraint information for an Autopilot action.
	 * 
	 * Constraints are max velocity, acceleration, and jerk.
	 */
	public static class APConstraints {
		protected double velocity;
		protected double acceleration;
		protected double jerk;
	
		/** Creates a blank APConstraints object.
		 * <p> A blank APConstraints will not limit velocity, acceleration, or jerk.
		*/
		public APConstraints() {
			this.velocity = Double.POSITIVE_INFINITY; // Default to no limit on velocity
		}
	
		/**
		 * Creates a new APConstraints object with a given max velocity, acceleration, and jerk.
		 * @param velocity The maximum velocity that Autopilot will demand, in m/s
		 * @param acceleration The maximum acceleration that Autopilot action will use to correct initial velocities, in m/s^2
		 * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
		 */
	
		public APConstraints(double velocity, double acceleration, double jerk) {
			this.velocity = velocity;
			this.acceleration = acceleration;
			this.jerk = jerk;
		}
	
		/**
		 * Create a new APConstraints object with a given max acceleration and jerk.
		 * <p> This constructor defaults the velocity to unlimited.
		 * @param acceleration The maximum acceleration that  Autopilot action will use to correct initial velocities, in m/s^2
		 * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
		 * 
		 */
		public APConstraints(double acceleration, double jerk) {
			this.acceleration = acceleration;
			this.jerk = jerk;
			this.velocity = Double.POSITIVE_INFINITY;
		}
	
		/**
		 * Modifies this APConstraints object's max velocity and returns itself. 
		 * This affects the maximum velocity that Autopilot can demand.
		 * 
		 * @param velocity The maximum velocity that Autopilot will demand, in m/s
		 */
		public APConstraints withVelocity(double velocity) {
			this.velocity = velocity;
			return this;
		}
	
		/**
		 * Modifies this APConstraint object's acceleration and returns itself. This affects the maximum
		 * acceleration that Autopilot will use to correct initial velocities.
		 *
		 * <p> Autopilot's acceleration is used at the beginning of an action (not relevant to Autopilot's end behavior).
		 * 
		 * @param acceleration The maximum acceleration that Autopilot will use to start a path, in m/s^2
		 */
		public APConstraints withAcceleration(double acceleration) {
			this.acceleration = acceleration;
			return this;
		}
	
		/**
		 * Modifies this constraint's max jerk value and returns itself. Higher values mean a faster
		 * deceleration.
		 * 
		 * Autopilot's jerk is used at the end of an action (not relevant to Autopilot's start behavior).
		 * 
		 * @param jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in m/s^3
		 */
		public APConstraints withJerk(double jerk) {
			this.jerk = jerk;
			return this;
		}
	}
	
	/**
	 * A class representing a profile that determines how Autopilot approaches a target.
	 *
	 * The constraints property of the profile limits the robot's behavior.
	 *
	 * <p> Acceptable error for the controller (both translational and rotational) are stored here.
	 *
	 * <p> The "beeline radius" determines the distance at which the robot drives directly at the target and
	 * no longer respects entry angle. This is helpful because if the robot overshoots by a small
	 * amount, that error should not cause the robot do completely circle back around.
	 */
	public static class APProfile {
		protected APConstraints constraints;
		protected Distance errorXY;
		protected Angle errorTheta;
		protected Distance beelineRadius;

		/**
		 * Builds an APProfile with the given constraints. Tolerated error and beeline radius are all set
		 * to zero.
		 *
		 * @param constraints The motion constraints for this profile
		 */
		public APProfile(APConstraints constraints) {
			this.constraints = constraints;
			errorXY = Meters.of(0);
			errorTheta = Rotations.of(0);
			beelineRadius = Meters.of(0);
		}

		/**
		 * Modifies this profile's tolerated error in the XY plane and returns itself
		 * 
		 * @param errorXY The tolerated translation error for this profile
		 */
		public APProfile withErrorXY(Distance errorXY) {
			this.errorXY = errorXY;
			return this;
		}

		/**
		 * Modifies this profile's tolerated angular error and returns itself
		 * 
		 * @param errorTheta The tolerated angular error for this profile
		 */
		public APProfile withErrorTheta(Angle errorTheta) {
			this.errorTheta = errorTheta;
			return this;
		}

		/**
		 * Modifies this profile's path generation constraints and returns itself
		 * 
		 * @param constraints The Autopilot constraints to apply to this profile
		 */
		public APProfile withConstraints(APConstraints constraints) {
			this.constraints = constraints;
			return this;
		}

		/**
		 * Modifies this profile's beeline radius and returns itself
		 *
		 * <p> The beeline radius is a distance where, under that range, entry angle is no longer respected.
		 * This prevents small overshoots from causing the robot to make a full arc and instead correct
		 * itself.
		 * 
		 * @param beelineRadius The distance at which the robot will drive directly at the target
		 */
		public APProfile withBeelineRadius(Distance beelineRadius) {
			this.beelineRadius = beelineRadius;
			return this;
		}

		/**
		 * Returns the tolerated translation error for this profile.
		 */
		public Distance getErrorXY() {
			return errorXY;
		}

		/**
		 * Returns the tolerated angular error for this profile.
		 */
		public Angle getErrorTheta() {
			return errorTheta;
		}

		/**
		 * Returns the path generation constraints for this profile.
		 */
		public APConstraints getConstraints() {
			return constraints;
		}

		/**
		 * Returns the beeline radius for this profile.
		 */
		public Distance getBeelineRadius() {
			return beelineRadius;
		}
	}
	
	/**
	 * The APTarget class represents the goal end state of an Autopilot action.
	 * 
	 * A target needs a reference Pose2d, but can optionally have a specified entry angle and rotation
	 * radius.
	 *
	 * A target may also specify an end velocity or end velocity.
	 */
	public static class APTarget {
		protected Pose2d m_reference;
		protected Optional<Rotation2d> m_entryAngle;
		protected double m_velocity;
		protected Optional<Distance> m_rotationRadius;

		/**
		 * Creates a new Autopilot target with the given target pose, no entry angle, and no end velocity.
		 * 
		 * @param pose The reference pose for this target.
		 */
		public APTarget(Pose2d pose) {
			m_reference = pose;
			m_velocity = 0;
			m_entryAngle = Optional.empty();
			m_rotationRadius = Optional.empty();
		}

		/**
		 * Returns a copy of this target with the given reference Pose2d.
		 *
		 * @param reference The reference Pose2d for this target.
		 */
		public APTarget withReference(Pose2d reference) {
			APTarget target = this.clone();
			target.m_reference = reference;
			return target;
		}

		/**
		 * Returns a copy of this target with the given entry angle.
		 *
		 * @param entryAngle The entry angle for the new target.
		 */
		public APTarget withEntryAngle(Rotation2d entryAngle) {
			APTarget target = this.clone();
			target.m_entryAngle = Optional.of(entryAngle);
			return target;
		}

		/**
		 * Returns a copy of this target with the given end velocity. Note that if the robot does not
		 * reach this velocity, no issues will be thrown. Autopilot will only try to reach this target,
		 * but it will not be affected by whether it does.
		 *
		 * @param velocity The desired end velocity when the robot approaches the target
		 */
		public APTarget withVelocity(double velocity) {
			APTarget target = this.clone();
			target.m_velocity = velocity;
			return target;
		}

		/**
		 * Returns a copy of this target with the given rotation radius.
		 *
		 * <p> Rotation radius is the distance from the target pose that rotation goals are respected.
		 * 
		 * <p> By default, rotation goals are always respected. Adjusting this radius prevents Autopilot from reorienting
		 * the robot until the robot is within the specified radius of the target.
		 *
		 * @param radius The rotation radius for the new target
		 */
		public APTarget withRotationRadius(Distance radius) {
			APTarget copy = this.clone();
			copy.m_rotationRadius = Optional.of(radius);
			return copy;
		}

		/**
		 * Returns this target's reference Pose2d.
		 */
		public Pose2d getReference() {
			return m_reference;
		}

		/**
		 * Returns this target's desired entry angle.
		 */
		public Optional<Rotation2d> getEntryAngle() {
			return m_entryAngle;
		}

		/**
		 * Returns this target's end velocity.
		 */
		public double getVelocity() {
			return m_velocity;
		}

		/**
		 * Returns this target's rotation radius.
		 */
		public Optional<Distance> getRotationRadius() {
			return m_rotationRadius;
		}

		/**
		 * Creates a copy of this APTarget.
		 */
		public APTarget clone() {
			APTarget target = new APTarget(m_reference);
			target.m_velocity = m_velocity;
			target.m_entryAngle = m_entryAngle;
			target.m_rotationRadius = m_rotationRadius;
			return target;
		}

		/**
		 * Retuns a copy of this target, without the entry angle set. 
		 * 
		 * <p> This is useful if trying to make two different targets with and without entry angle set.
		 */
		public APTarget withoutEntryAngle() {
			APTarget target = new APTarget(m_reference);
			target.m_velocity = m_velocity;
			target.m_rotationRadius = m_rotationRadius;
			return target;
		}
	}

	/**
	 * Constructs an Autopilot from a given profile. This is the profile that the autopilot will use
	 * for all actions.
	 */
	public Autopilot(APProfile profile) {
		m_profile = profile;
	}

	/**
	 * Returns the next field relative velocity for the trajectory
	 *
	 * @param current The robot's current position.
	 * @param robotRelativeSpeeds The robot's current <b>robot relative</b> ChassisSpeeds.
	 * @param target The target the robot should drive towards.
	 * 
	 * @return an APResult containing the next velocity and target angle
	 */
	public APResult calculate(Pose2d current, ChassisSpeeds robotRelativeSpeeds, APTarget target) {
		Translation2d offset = toTargetCoordinateFrame(
				target.m_reference.getTranslation().minus(current.getTranslation()), target);

		if (offset.equals(Translation2d.kZero)) {
			return new APResult(
					MetersPerSecond.zero(),
					MetersPerSecond.zero(),
					target.m_reference.getRotation());
		}

		Translation2d fieldRelativeSpeeds = new Translation2d(
				robotRelativeSpeeds.vxMetersPerSecond,
				robotRelativeSpeeds.vyMetersPerSecond).rotateBy(current.getRotation());

		Translation2d initial = toTargetCoordinateFrame(fieldRelativeSpeeds, target);
		double disp = offset.getNorm();
		if (target.m_entryAngle.isEmpty() || disp < m_profile.beelineRadius.in(Meters)) {
			Translation2d towardsTarget = offset.div(disp);
			Translation2d goal = towardsTarget.times(calculateMaxVelocity(disp, target.m_velocity));
			Translation2d out = correct(initial, goal);
			Translation2d velo = toGlobalCoordinateFrame(out, target);
			Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
			return new APResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
		}
		Translation2d goal = calculateSwirlyVelocity(offset, target);
		Translation2d out = correct(initial, goal);
		Translation2d velo = toGlobalCoordinateFrame(out, target);
		Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
		return new APResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
	}

	/**
	 * Turns any other coordinate frame into a coordinate frame with positive x meaning in the
	 * direction of the target's entry angle, if applicable (otherwise no change to angles).
	 */
	private Translation2d toTargetCoordinateFrame(Translation2d coords, APTarget target) {
		Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
		return coords.rotateBy(entryAngle.unaryMinus());
	}

	/**
	 * Turns a translation from a target-relative coordinate frame to a global coordinate frame.
	 */
	private Translation2d toGlobalCoordinateFrame(Translation2d coords, APTarget target) {
		Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
		return coords.rotateBy(entryAngle);
	}

	/**
	 * Determines the maximum velocity required to travel the given distance and end at the desired
	 * end velocity.
	 * @param dist The distance to travel, in meters
	 * @param endVelo The desired end velocity, in m/s
	 */
	private double calculateMaxVelocity(double dist, double endVelo) {
		return Math.pow((4.5 * Math.pow(dist, 2.0)) * m_profile.constraints.jerk, 1.0 / 3.0)
				+ endVelo;
	}

	/**
	 * Attempts to drive the initial translation to the goal translation using the parameters for
	 * acceleration given in the profile.
	 * 
	 * @param initial The initial translation to drive from
	 * @param goal The goal translation to drive to
	 */
	private Translation2d correct(Translation2d initial, Translation2d goal) {
		Rotation2d angleOffset = Rotation2d.kZero;
		if (!goal.equals(Translation2d.kZero)) {
			angleOffset = new Rotation2d(goal.getX(), goal.getY());
		}
		Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
		Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());
		double initialI = adjustedInitial.getX();
		double goalI = adjustedGoal.getX();
		// we cap the adjusted I because we'd rather adjust now than overshoot.
		if (goalI > m_profile.constraints.velocity) {
			goalI = m_profile.constraints.velocity;
		}
		double adjustedI = Math.min(goalI,
				push(initialI, goalI, m_profile.constraints.acceleration));
		return new Translation2d(adjustedI, 0).rotateBy(angleOffset);
	}

	/**
	 * Using the provided acceleration, "pushes" the start point towards the end point.
	 *
	 * This is used for ensuring that changes in velocity are withing the acceleration threshold.
	 */
	private double push(double start, double end, double accel) {
		double maxChange = accel * dt;
		if (Math.abs(start - end) < maxChange) {
			return end;
		}
		if (start > end) {
			return start - maxChange;
		}
		return start + maxChange;
	}

	/**
	 * Uses the swirly method to calculate the correct velocities for the robot, respecting entry
	 * angles.
	 *
	 * @param offset The offset from the robot to the target, in the target's coordinate frame
	 * @param target The target that Autopilot is trying to reach
	 */
	private Translation2d calculateSwirlyVelocity(Translation2d offset, APTarget target) {
		double disp = offset.getNorm();
		Rotation2d theta = new Rotation2d(offset.getX(), offset.getY());
		double rads = theta.getRadians();
		double dist = calculateSwirlyLength(rads, disp);
		double vx = theta.getCos() - rads * theta.getSin();
		double vy = rads * theta.getCos() + theta.getSin();
		return new Translation2d(vx, vy)
				.div(Math.hypot(vx, vy)) // normalize
				.times(calculateMaxVelocity(dist, target.m_velocity)); // and scale to new length
	}

	/**
	 * Using a precomputed integral, returns the length of the path that the swirly method generates.
	 *
	 * <p> More specifically, this calculates the arc length of the polar curve r=theta from the given
	 * angle to zero, then scales it to match the current state.
	 * 
	 * @param theta The angle of the offset from the robot to the target, in radians
	 * @param radius The normalized offset from the robot to the target, in meters
	 */
	private double calculateSwirlyLength(double theta, double radius) {
		if (theta == 0) {
			return radius;
		}
		theta = Math.abs(theta);
		double hypot = Math.hypot(theta, 1);
		double u1 = radius * hypot;
		double u2 = radius * Math.log(theta + hypot) / theta;
		return 0.5 * (u1 + u2);
	}

	/**
	 * Returns the target's rotation if the robot is within a specified rotation radius; otherwise, returns the current rotation of the robot.
	 * 
	 * @param current The current rotation of the robot.
	 * @param target The APTarget that Autopilot is trying to reach.
	 * @param dist The distance from the robot to the target.
	 */
	private Rotation2d getRotationTarget(Rotation2d current, APTarget target, double dist) {
		if (target.m_rotationRadius.isEmpty()) {
			return target.m_reference.getRotation();
		}
		double radius = target.m_rotationRadius.get().in(Meters);
		if (radius > dist) {
			return target.m_reference.getRotation();
		} else {
			return current;
		}
	}

	/**
	 * Return whether the given pose is within the tolerance of the APTarget.
	 * 
	 * @param current The current pose of the robot.
	 * @param target The APTarget to check against.
	 * 
	 * @return Returns true if Autopilot has reached the target.
	 */
	public boolean atTarget(Pose2d current, APTarget target) {
		Pose2d goal = target.m_reference;
		boolean okXY = Math.hypot(current.getX() - goal.getX(),
				current.getY() - goal.getY()) <= m_profile.errorXY.in(Meters);
		boolean okTheta = Math.abs(current.getRotation().minus(goal.getRotation())
				.getRadians()) <= m_profile.errorTheta.in(Radians);
		return okXY && okTheta;
	}

	/**
	 * The computed motion from a call to <code>Autopilot.calculate()</code>.
	 */
	public record APResult(LinearVelocity vx, LinearVelocity vy, Rotation2d targetAngle) {}
}