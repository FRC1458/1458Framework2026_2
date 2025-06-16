package frc.robot.lib.control;

/** 
 * An interface for code that drives a system to a target setpoint.
 */
public interface Controller<I, O> extends IO<I, O> {
    /**
     * Sets the target.
     * @param target The target to try to reach.
     */
    public void setTarget(O target);
}
