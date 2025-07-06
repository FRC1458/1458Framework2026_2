package frc.robot.lib.drivers;

public class CanDeviceId {
    private final int deviceNumber;
    private final String bus;

    public CanDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        this.bus = bus;
    }

    // Use the default bus name (empty string).
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() { return deviceNumber; }

    public String getBus() { return bus; }

    public boolean equals(CanDeviceId other) {
        return other.deviceNumber == deviceNumber && other.bus == bus;
    }
}
