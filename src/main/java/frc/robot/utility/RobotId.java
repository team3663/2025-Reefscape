package frc.robot.utility;

public enum RobotId {
    UNKNOWN("00-00-00-00-00-00"),
    C2024("00-80-2f-33-d0-1c"),
    C2025("00-80-2f-33-d0-3f");

    private final String macAddress;

    RobotId(String macAddress) {
        this.macAddress = macAddress;
    }

    public String getMacAddress() {
        return macAddress;
    }
}
