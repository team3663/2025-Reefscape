package frc.robot.subsystems.led;

import edu.wpi.first.epilogue.Logged;

@Logged
public class LedColor {
    public int red;
    public int green;
    public int blue;

    public LedColor(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
}