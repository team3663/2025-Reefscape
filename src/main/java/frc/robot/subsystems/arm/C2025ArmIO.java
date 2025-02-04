package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class C2025ArmIO implements ArmIO {
    // TODO: Get real values from CAD
    private static final Arm.Constants CONSTANTS = new Arm.Constants(
            0.2, Units.degreesToRadians(-135.0), Units.degreesToRadians(180.0),
            0.05, Units.degreesToRadians(-90.0), Units.degreesToRadians(90.0));

    private final TalonFX shoulderMotor;
    private final TalonFX wristMotor;

    private final DCMotorSim shoulderSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

    private final DCMotorSim wristSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2025ArmIO(TalonFX shoulderMotor, TalonFX wristMotor) {
        this.shoulderMotor = shoulderMotor;
        this.wristMotor = wristMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kV = 0.115;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.79;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;

        shoulderMotor.getConfigurator().apply(config);
        wristMotor.getConfigurator().apply(config);
    }

    @Override
    public Arm.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        // Sims for Wrist and Shoulder
        if (Robot.isSimulation()) {
            // Shoulder sim
            var shoulderSimState = shoulderMotor.getSimState();
            shoulderSim.setInputVoltage(shoulderSimState.getMotorVoltage());

            // Updates sim for shoulder every 20 milliseconds
            shoulderSim.update(Robot.kDefaultPeriod);
            shoulderSimState.setRotorAcceleration(shoulderSim.getAngularAcceleration());
            shoulderSimState.setRotorVelocity(shoulderSim.getAngularVelocity());
            shoulderSimState.setRawRotorPosition(shoulderSim.getAngularPosition());

            // Wrist sim
            var wristSimState = wristMotor.getSimState();
            wristSim.setInputVoltage(wristSimState.getMotorVoltage());

            // Updates sim for wrist every 20 milliseconds
            wristSim.update(Robot.kDefaultPeriod);
            wristSimState.setRotorAcceleration(wristSim.getAngularAcceleration());
            wristSimState.setRotorVelocity(wristSim.getAngularVelocity());
            wristSimState.setRawRotorPosition(wristSim.getAngularPosition());
        }

        // Wrist inputs
        inputs.currentAppliedVoltageWrist = wristMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentVelocityWrist = Units.rotationsToRadians(wristMotor.getVelocity().getValueAsDouble());
        inputs.currentPositionWrist = Units.rotationsToRadians(wristMotor.getPosition().getValueAsDouble());
        inputs.motorTemperatureWrist = wristMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawWrist = wristMotor.getSupplyCurrent().getValueAsDouble();

        // Shoulder inputs
        inputs.currentAppliedVoltageShoulder = shoulderMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentVelocityShoulder = Units.rotationsToRadians(shoulderMotor.getVelocity().getValueAsDouble());
        inputs.currentPositionShoulder = Units.rotationsToRadians(shoulderMotor.getPosition().getValueAsDouble());
        inputs.motorTemperatureShoulder = shoulderMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawShoulder = shoulderMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void stopShoulder() {
        shoulderMotor.setControl(stopRequest);
    }

    @Override
    public void resetShoulderPosition() {
        shoulderMotor.setPosition(0.0);
    }

    @Override
    public void setShoulderTargetPosition(double position) {
        shoulderMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void stopWrist() {
        wristMotor.setControl(stopRequest);
    }

    @Override
    public void resetWristPosition() {
        wristMotor.setPosition(0.0);
    }

    @Override
    public void setWristTargetPosition(double position) {
        wristMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }
}