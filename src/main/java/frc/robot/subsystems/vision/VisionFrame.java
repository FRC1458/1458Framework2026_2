package frc.robot.subsystems.vision;

public class VisionFrame {
	double timestamp;
	double[] frame_data;
	double[] stdDevs;

	// For comparator
	public double getTimestamp() {
		return timestamp;
	}
}
