package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ClimberInputs {
    public double currentPosition;
    public double targetPosition;
    public double targetVoltage;
    public double currentAppliedVoltage;
    public double motorTemperature;
    public double currentDraw;
    public boolean gamePieceDetected1;
    public boolean gamePieceDetected2;
}
