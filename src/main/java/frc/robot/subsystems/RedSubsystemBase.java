package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.*;

/**
 * A custom subsystem base.
 */
public abstract class RedSubsystemBase implements Subsystem, Sendable {
	public String name;
	/** Constructor. Telemetry/log name defaults to the classname. */
	@SuppressWarnings("this-escape")
	public RedSubsystemBase() {
		this.name = this.getClass().getSimpleName();
		this.name = name.substring(name.lastIndexOf('.') + 1);
	}

	/**
	 * Constructor.
	 *
	 * @param name Name of the subsystem for telemetry and logging.
	 */
	@SuppressWarnings("this-escape")
	public RedSubsystemBase(String name) {
		this.name = name;
	}

	/**
	 * Gets the name of this Subsystem.
	 *
	 * @return Name
	 */
	@Override
	public String getName() {
		return SendableRegistry.getName(this);
	}

	/**
	 * Sets the name of this Subsystem.
	 *
	 * @param name name
	 */
	public void setName(String name) {
		SendableRegistry.setName(this, name);
	}

	/**
	 * Gets the subsystem name of this Subsystem.
	 *
	 * @return Subsystem name
	 */
	public String getSubsystem() {
		return SendableRegistry.getSubsystem(this);
	}

	/**
	 * Sets the subsystem name of this Subsystem.
	 *
	 * @param subsystem subsystem name
	 */
	public void setSubsystem(String subsystem) {
		SendableRegistry.setSubsystem(this, subsystem);
	}

	public void register() {
		SendableRegistry.addLW(this, name, name);
		CommandScheduler.getInstance().registerSubsystem(this);
	}

	/**
	 * Associates a {@link Sendable} with this Subsystem. Also update the child's name.
	 *
	 * @param name name to give child
	 * @param child sendable
	 */
	public void addChild(String name, Sendable child) {
		SendableRegistry.addLW(child, getSubsystem(), name);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Subsystem");

		builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
		builder.addStringProperty(
			".default",
			() -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
			null);
		builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
		builder.addStringProperty(
			".command",
			() -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
			null);
	}
}
