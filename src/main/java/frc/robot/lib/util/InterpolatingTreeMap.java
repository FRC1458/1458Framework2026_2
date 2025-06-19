package frc.robot.lib.util;

import java.util.Comparator;
import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess
 * from points that are defined. This uses linear interpolation.
 *
 * <p>{@code K} must implement {@link Comparable}, or a {@link Comparator} on {@code K} can be
 * provided.
 *
 * @param <K> The type of keys held in this map.
 * @param <V> The type of values held in this map.
 */
public class InterpolatingTreeMap<K, V> {
	private final TreeMap<K, V> map;

	private final InverseInterpolator<K> inverseInterpolator;
	private final Interpolator<V> interpolator;

	private final int max;

	/**
	 * Constructs an InterpolatingTreeMap.
	 *
	 * @param inverseInterpolator Function to use for inverse interpolation of the keys.
	 * @param interpolator Function to use for interpolation of the values.
	 */
	public InterpolatingTreeMap(
		InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator, int max) {
		map = new TreeMap<>();
		this.inverseInterpolator = inverseInterpolator;
		this.interpolator = interpolator;
		this.max = max;
	}

	/**
	 * Constructs an InterpolatingTreeMap using {@code comparator}.
	 *
	 * @param inverseInterpolator Function to use for inverse interpolation of the keys.
	 * @param interpolator Function to use for interpolation of the values.
	 * @param comparator Comparator to use on keys.
	 */
	public InterpolatingTreeMap(
		InverseInterpolator<K> inverseInterpolator,
		Interpolator<V> interpolator,
		Comparator<K> comparator,
		int max) {
		this.inverseInterpolator = inverseInterpolator;
		this.interpolator = interpolator;
		map = new TreeMap<>(comparator);
		this.max = max;
	}

	/**
	 * Inserts a key-value pair.
	 *
	 * @param key The key.
	 * @param value The value.
	 */
	public void put(K key, V value) {
		map.put(key, value);
		if (map.size() > max) {
			map.remove(map.firstEntry());
		}
	}

	/**
	 * Returns the value associated with a given key.
	 *
	 * <p>If there's no matching key, the value returned will be an interpolation between the keys
	 * before and after the provided one, using the {@link Interpolator} and {@link
	 * InverseInterpolator} provided.
	 *
	 * @param key The key.
	 * @return The value associated with the given key.
	 */
	public V get(K key) {
		V val = map.get(key);
		if (val == null) {
		K ceilingKey = map.ceilingKey(key);
		K floorKey = map.floorKey(key);

		if (ceilingKey == null && floorKey == null) {
			return null;
		}
		if (ceilingKey == null) {
			return map.get(floorKey);
		}
		if (floorKey == null) {
			return map.get(ceilingKey);
		}
		V floor = map.get(floorKey);
		V ceiling = map.get(ceilingKey);

		return interpolator.interpolate(
			floor, ceiling, inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
		} else {
			return val;
		}
	}

	/** Clears the contents. */
	public void clear() {
		map.clear();
	}

	public Entry<K, V> lastEntry() {
		return map.lastEntry();
	}

	public boolean isEmpty() {
		return map.isEmpty();
	}

	public K lastKey() {
		return map.lastKey();
	}
}
