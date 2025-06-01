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

    // Continuous-input support
    private boolean isContinuousInputEnabled = false;
    private double mMinInput = 0.0;
    private double mMaxInput = 0.0;

    private final Timer mDtTracker = new Timer();

    public PIDController(PIDFConstants constants) {
        this.mConstants = constants;
        mDtTracker.start();
    }

    public PIDController(PIDConstants constants) {
        this(new PIDFConstants(constants));
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        isContinuousInputEnabled = true;
        mMinInput = minimumInput;
        mMaxInput = maximumInput;
    }

    @Override
    public void setInput(Double input) {
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
        if (dt <= 0.0) {
            return 0.0;
        }

        if (isContinuousInputEnabled) {
            double range = mMaxInput - mMinInput;
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

    public void setIntegral(double integral) {
        this.integral = integral;
    }

    public void reset() {
        integral = 0.0;
        error = 0.0;
        prevError = 0.0;
        feedforward = 0.0;
        mDtTracker.reset();
        mDtTracker.start();
    }    
}
