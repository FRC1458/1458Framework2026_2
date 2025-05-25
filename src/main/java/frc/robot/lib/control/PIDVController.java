package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.control.ControlConstants.*;

public class PIDVController implements Controller<Pair<Double, Double>, Double> {
    private final PIDFConstants mConstants;

    private double target = 0.0;
    private Pair<Double, Double> measurement = new Pair<>(0.0, 0.0);
    private double feedforward = 0.0;
    private double integral = 0.0;

    public double error = 0.0;

    private boolean isContinuous = false;
    private double minRange = 0.0;
    private double maxRange = 0.0;

    private final Timer mDtTracker = new Timer();

    public PIDVController(PIDFConstants constants) {
        this.mConstants = constants;
        mDtTracker.start();
    }

    public PIDVController(PIDConstants constants) {
        this(new PIDFConstants(constants));
    }

    /** Enable wrapping so error is always the shortest path around a circular range */
    public void enableContinuousInput(double minInput, double maxInput) {
        isContinuous = true;
        minRange = minInput;
        maxRange = maxInput;
    }

    /** Turn off continuous-input wrapping */
    public void disableContinuousInput() {
        isContinuous = false;
    }

    @Override
    public void setInput(Pair<Double, Double> input) {
        this.measurement = input;
    }

    @Override
    public void setTarget(Double target) {
        this.target = target;
    }

    public void setFeedforward(Double feedforward) {
        this.feedforward = feedforward;
    }

    @Override
    public Double getOutput() {
        double dt = mDtTracker.get();
        mDtTracker.reset();
        if (dt <= 0.0) return 0.0;
        
        double position = measurement.getFirst();
        double velocity = measurement.getSecond();

        error = isContinuous
            ? MathUtil.inputModulus(target - position, -(maxRange - minRange) / 2.0, (maxRange - minRange) / 2.0)
            : target - position;

        integral += error * dt;

        double derivative = -velocity;

        return mConstants.kP * error
             + mConstants.kI * integral
             + mConstants.kD * derivative
             + mConstants.kF * feedforward;
    }

    public void setIntegral(double integral) {
        this.integral = integral;
    }
}
