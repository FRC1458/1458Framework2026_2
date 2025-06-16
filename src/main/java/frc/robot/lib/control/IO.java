package frc.robot.lib.control;

/** An interface for systems that map an input of type {@code I} to an output of type {@code O}. */
public interface IO<I, O> {
    /** Sets the input.
     * @param input The input to set.
     */
    public void setInput(I input);

    /**
     * Gets the output.
     * @return The output of the IO.
     */
    public O getOutput();
}
