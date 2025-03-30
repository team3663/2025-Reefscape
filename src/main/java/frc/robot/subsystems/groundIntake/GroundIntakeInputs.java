package frc.robot.subsystems.groundIntake;

import edu.wpi.first.epilogue.Logged;

@Logged
public class GroundIntakeInputs {
    // Pivot Motor
    public double pivotCurrentVelocity;
    public double pivotCurrentAppliedVoltage;
    public double pivotCurrentPosition;
    public double pivotMotorTemperature;
    public double pivotCurrentDraw;

    // Intake Motor
    public double intakeCurrentVelocity;
    public double intakeCurrentAppliedVoltage;
    public double intakeMotorTemperature;
    public double intakeCurrentDraw;

    public boolean gamePieceDetected;
}
