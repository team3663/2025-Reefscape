package frc.robot.subsystem.grabber;

import edu.wpi.first.epilogue.Logged;

@Logged
public class GrabberInputs {
    public double currentVelocity;
    public double currentAppliedVoltage;
    public double currentPosition;

    public double motorTemperature;
    public double currentDraw;

    public boolean beamBreakState;
}