package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ArmInputs {
    // Shoulder inputs
    public double currentShoulderVelocity;
    public double currentAppliedShoulderVoltage;
    public double currentShoulderPosition;
    
    public double shoulderMotorPosition;
    public double currentShoulderDraw;

    // Wrist Inputs
    public double currentWristVelocity;
    public double currentWristAppliedVoltage;
    public double currentWristPosition;

    public double wristMotorPosition;
    public double currentWristDraw;
}