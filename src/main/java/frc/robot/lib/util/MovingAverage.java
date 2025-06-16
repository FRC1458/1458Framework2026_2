package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.function.BiFunction;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage<T> {
    ArrayList<T> current = new ArrayList<T>();
    int maxSize;
    BiFunction<T, T, T> addFunction;
    BiFunction<T, Integer, T> divideFunction;
    T zero;

    /**
     * Generates a MovingAverage object
     * @param maxSize The maximum size of the moving average.
     * @param zero The zero element of type T.
     * @param add The addition function of type T.
     * @param divide The scalar division function of type T.
     */
    public MovingAverage(int maxSize, T zero, BiFunction<T, T, T> add, BiFunction<T, Integer, T> divide) {
        this.maxSize = maxSize;
        this.addFunction = add;
        this.divideFunction = divide;
        this.zero = zero;
    }

    /**
     * Adds a value to the moving average, popping the first value if it exceeds the max size.
     * @param other The value to add.
     */
    public void add(T other) {
        current.add(other);
        if (current.size() > maxSize) {
            current.remove(0);
        }
    }

    /**
     * Gets the moving average.
     * @return The moving average.
     */
    public T getAverage() {
        T total = zero;

        for (T obj : current) {
            total = addFunction.apply(total, obj);
        }

        return divideFunction.apply(total, current.size());
    }

    /**
     * Gets the current size of the moving average.
     * @return The current size.
     */
    public int getSize() {
        return current.size();
    }

    /**
     * Whether the moving average is under max capacity.
     * @return Whether the moving average is under max capacity.
     */
    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    /**
     * Clears the current moving average.
     */
    public void clear() {
        current.clear();
    }
}