package frc.robot.lib.control;

public interface Controller<I, O> extends IO<I, O> {
    public void setTarget(O target);
}
