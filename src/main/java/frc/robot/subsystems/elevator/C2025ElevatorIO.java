package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class C2025ElevatorIO implements ElevatorIO {
    // TODO: Get real values from CAD
    private static final Elevator.Constants CONSTANTS = new Elevator.Constants(0.0, 1.0);

    private static final double GEAR_RATIO = 1.0;
    private static final double PULLEY_RADIUS = Units.inchesToMeters(1.0);
    private static final double PULLEY_CIRCUMFERENCE = (2 * Math.PI * PULLEY_RADIUS);

    private final TalonFX motor;
    private final TalonFX motor2;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2),
                    0.001, GEAR_RATIO),
            DCMotor.getKrakenX60(2).withReduction(1.0)
    );

    public C2025ElevatorIO(TalonFX motor, TalonFX motor2) {
        this.motor = motor;
        this.motor2 = motor2;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kV = 0.115;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;
        config.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;

        motor.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(motor.getDeviceID(), true));
    }

    @Override
    public Elevator.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        if (Robot.isSimulation()) {
            // Sims for Motor 1 and 2
            var simStateMotor1 = motor.getSimState();
            var simStateMotor2 = motor2.getSimState();
            sim.setInputVoltage(simStateMotor1.getMotorVoltage());
            // Updates the sim information every 20 ms
            sim.update(Robot.kDefaultPeriod);
            simStateMotor1.setRotorAcceleration(sim.getAngularAcceleration());
            simStateMotor1.setRotorVelocity(sim.getAngularVelocity());
            simStateMotor1.setRawRotorPosition(sim.getAngularPosition());
            simStateMotor2.setRotorAcceleration(sim.getAngularAcceleration());
            simStateMotor2.setRotorVelocity(sim.getAngularVelocity());
            simStateMotor2.setRawRotorPosition(sim.getAngularPosition());
        }

        // Inputs for Motor 1
        inputs.currentVelocityMotor1 = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor1 = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentPositionMotor1 = motor.getPosition().getValueAsDouble() * PULLEY_CIRCUMFERENCE;
        inputs.motorTemperatureMotor1 = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor1 = motor.getSupplyCurrent().getValueAsDouble();

        // Inputs for Motor 2
        inputs.currentVelocityMotor2 = Units.rotationsToRadians(motor2.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor2 = motor2.getMotorVoltage().getValueAsDouble();
        inputs.currentPositionMotor2 = motor.getPosition().getValueAsDouble() * PULLEY_CIRCUMFERENCE;
        inputs.motorTemperatureMotor2 = motor2.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor2 = motor2.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        motor.setPosition(0);
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetPosition(double position) {
        motor.setControl(positionRequest.withPosition(position / PULLEY_CIRCUMFERENCE));
    }
}