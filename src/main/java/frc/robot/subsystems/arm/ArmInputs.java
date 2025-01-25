package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ArmInputs {
    // Shoulder inputs
    public double currentVelocityShoulder;
    public double currentAppliedVoltageShoulder;
    public double currentPositionShoulder;
    
    public double motorTemperatureShoulder;
    public double currentDrawShoulder;

    // Wrist Inputs
    public double currentVelocityWrist;
    public double currentAppliedVoltageWrist;
    public double currentPositionWrist;

    public double motorTemperatureWrist;
    public double currentDrawWrist;
}