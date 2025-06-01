package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void enableContinuousInput(double minInput, double maxInput) {
        isContinuous = true;
        minRange = minInput;
        maxRange = maxInput;
    }

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

    public void reset() {
        integral = 0.0;
        feedforward = 0.0;
        error = 0.0;
        mDtTracker.reset();
        mDtTracker.start();
    }
}
