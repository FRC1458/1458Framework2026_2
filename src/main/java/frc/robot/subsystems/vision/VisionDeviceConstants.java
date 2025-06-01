package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;

public class VisionDeviceConstants {
	public String kTableName = "limelight-front";//"limelight-c";
	public Transform2d kRobotToCamera = new Transform2d();
	public int kCameraId = 0;
	public int kCameraResolutionWidth = 1600;
	public int kCameraResolutionHeight = 1200;
}
