package frc.robot.subsystems.grabber;

import edu.wpi.first.epilogue.Logged;

@Logged
public class GrabberInputs {
    public double currentVelocity;
    public double currentAppliedVoltage;

    public double motorTemperature;
    public double currentDraw;

    public boolean gamePieceDetected;
    public boolean gamePieceDetectedPrevious;
}