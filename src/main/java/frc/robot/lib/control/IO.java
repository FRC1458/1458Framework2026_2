package frc.robot.lib.control;

public interface IO<I, O> {
    public void setInput(I input);
    public O getOutput();
}
