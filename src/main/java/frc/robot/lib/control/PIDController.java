package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.control.ControlConstants.*;

public class PIDController implements Controller<Double, Double> {
    private final PIDFConstants mConstants;

    private double target = 0.0;
    private double measurement = 0.0;
    private double feedforward = 0.0;

    private double error = 0.0;
    private double prevError = 0.0;
    private double integral = 0.0;

    private boolean isContinuousInputEnabled = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    private final Timer mDtTracker = new Timer();

    /** 
     * A PID(F) controller. It works by acting like a dampened string, pulling the output towards a target point.
     * @param constants The {@link PIDFConstants}.
     */
    public PIDController(PIDFConstants constants) {
        this.mConstants = constants;
        mDtTracker.start();
    }
    /** 
     * A PID controller. It works by acting like a dampened string, pulling the output towards a target point.
     * @param constants The {@link PIDConstants}.
     */
    public PIDController(PIDConstants constants) {
        this(new PIDFConstants(constants));
    }
    /**
     * Makes the controller continuous, which means that values repeat.
     * @param minInput The minimum value.
     * @param maxInput The maximum value.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        isContinuousInputEnabled = true;
        minInput = minimumInput;
        maxInput = maximumInput;
    }

    /** Makes the controller discontinuous */
    public void disableContinuousInput() {
        isContinuousInputEnabled = false;
    }

    @Override
    public void setInput(Double input) {
        this.measurement = input;
    }

    @Override
    public void setTarget(Double target) {
        this.target = target;
    }

    /** Sets the feedforward value. */
    public void setFeedforward(Double feedforward) {
        this.feedforward = feedforward;
    }

    @Override
    public Double getOutput() {
        double dt = mDtTracker.get();
        mDtTracker.reset();
        if (dt <= 0.0) {
            return 0.0;
        }

        if (isContinuousInputEnabled) {
            double range = maxInput - minInput;
            double halfRange = range / 2.0;
            error = MathUtil.inputModulus(target - measurement, -halfRange, halfRange);
        } else {
            error = target - measurement;
        }

        integral += error * dt;

        double derivative = (error - prevError) / dt;
        prevError = error;

        return mConstants.kP * error
             + mConstants.kI * integral
             + mConstants.kD * derivative
             + mConstants.kF * feedforward;
    }

    /** Sets the integral value. */
    public void setIntegral(double integral) {
        this.integral = integral;
    }

    /** Resets the controller. */
    public void reset() {
        integral = 0.0;
        error = 0.0;
        prevError = 0.0;
        feedforward = 0.0;
        mDtTracker.reset();
        mDtTracker.start();
    }    
}
