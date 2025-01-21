package frc.robot.subsystems.shoulder;

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

public class C2025ShoulderIO implements ShoulderIO {
    private final TalonFX motor;
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2025ShoulderIO(TalonFX motor) {
        this.motor = motor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kV = 0.115;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShoulderInputs inputs) {
        //SimStates
        if (Robot.isSimulation()) {
            var simStateMotor = motor.getSimState();
            sim.setInputVoltage(simStateMotor.getMotorVoltage());
            //Updates sim every 20 milliseconds
            sim.update(Robot.kDefaultPeriod);
            simStateMotor.setRotorAcceleration(sim.getAngularAcceleration());
            simStateMotor.setRotorVelocity(sim.getAngularVelocity());
            simStateMotor.setRawRotorPosition(sim.getAngularPosition());
        }
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentVelocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.currentPosition = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        motor.setPosition(0);
    }

    @Override
    public void setTargetPosition(double position) {
        motor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }
}