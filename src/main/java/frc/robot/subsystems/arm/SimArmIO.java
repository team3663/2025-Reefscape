package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class SimArmIO implements ArmIO {
    private static final Arm.Constants CONSTANTS = new Arm.Constants(
            Units.inchesToMeters(20.0), Units.degreesToRadians(-135.0), Units.degreesToRadians(180.0),
            Units.inchesToMeters(6.0), Units.degreesToRadians(-90.0), Units.degreesToRadians(90.0));

    private static final double SHOULDER_REDUCTION = 10.0;
    private static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500Foc(1).withReduction(SHOULDER_REDUCTION);
    private static final double SHOULDER_MOMENT_OF_INERTIA = 0.5;

    private static final double WRIST_REDUCTION = 10.0;
    private static final DCMotor WRIST_MOTOR = DCMotor.getFalcon500Foc(1).withReduction(WRIST_REDUCTION);
    private static final double WRIST_MOMENT_OF_INERTIA = 0.05;

    private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
            SHOULDER_MOTOR, SHOULDER_REDUCTION, SHOULDER_MOMENT_OF_INERTIA,
            CONSTANTS.shoulderLength(), CONSTANTS.minimumShoulderAngle(), CONSTANTS.maximumShoulderAngle(),
            true, Units.degreesToRadians(90.0));

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            WRIST_MOTOR, WRIST_REDUCTION, WRIST_MOMENT_OF_INERTIA,
            CONSTANTS.wristLength(), CONSTANTS.minimumWristAngle(), CONSTANTS.maximumWristAngle(),
            true, 0.0);

    private final ProfiledPIDController shoulderController = new ProfiledPIDController(
            100.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(200.0),
            Units.rotationsPerMinuteToRadiansPerSecond(400.0)
    ));

    private final ProfiledPIDController wristController = new ProfiledPIDController(
            100.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(500.0),
            Units.rotationsPerMinuteToRadiansPerSecond(700.0)
    ));

    private double targetShoulderPosition = Double.NaN;
    private double targetShoulderVoltage = Double.NaN;

    private double targetWristPosition = Double.NaN;
    private double targetWristVoltage = Double.NaN;
    
    @Override
    public Arm.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        double shoulderVoltage = 0.0;
        if (Double.isFinite(targetShoulderPosition)) {
            shoulderVoltage = shoulderController.calculate(shoulderSim.getAngleRads(), targetShoulderPosition);
        } else if (Double.isFinite(targetShoulderVoltage)) {
            shoulderVoltage = targetShoulderVoltage;
        }
        shoulderSim.setInputVoltage(shoulderVoltage);

        shoulderSim.update(Robot.kDefaultPeriod);

        inputs.currentAppliedShoulderVoltage = shoulderSim.getInput().get(0, 0);
        inputs.currentShoulderPosition = shoulderSim.getAngleRads();
        inputs.currentShoulderVelocity = shoulderSim.getVelocityRadPerSec();

        double wristVoltage = 0.0;
        if (Double.isFinite(targetWristPosition)) {
            wristVoltage = wristController.calculate(wristSim.getAngleRads(), targetWristPosition);
        } else if (Double.isFinite(targetWristVoltage)) {
            wristVoltage = targetWristVoltage;
        }
        wristSim.setInputVoltage(wristVoltage);

        wristSim.update(Robot.kDefaultPeriod);

        inputs.currentWristAppliedVoltage = wristSim.getInput().get(0, 0);
        inputs.currentWristPosition= wristSim.getAngleRads();
        inputs.currentWristVelocity = wristSim.getVelocityRadPerSec();
    }

    @Override
    public void setShoulderTargetPosition(double position) {
        targetShoulderPosition = position;
        targetShoulderVoltage = Double.NaN;
    }

    @Override
    public void setShoulderTargetVoltage(double voltage) {
        targetShoulderPosition = Double.NaN;
        targetShoulderVoltage = voltage;
    }

    @Override
    public void setWristTargetPosition(double position) {
        targetWristPosition = position;
        targetWristVoltage = Double.NaN;
    }

    @Override
    public void setWristTargetVoltage(double voltage) {
        targetWristPosition = Double.NaN;
        targetWristVoltage = voltage;
    }
}
