package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class C2025GrabberIO implements GrabberIO {
    private final TalonFX motor;
    private final DigitalInput gamePieceDetector;

    private final DCMotorSim simMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0)
    );

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2025GrabberIO(TalonFX motor, DigitalInput gamePieceDetector) {
        this.motor = motor;
        this.gamePieceDetector = gamePieceDetector;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(GrabberInputs inputs) {
        // Sim for Motor
        if (Robot.isSimulation()) {
            var simStateMotor = motor.getSimState();
            simMotor.setInputVoltage(simStateMotor.getMotorVoltage());
            // Updates the sim information every 20 ms
            simMotor.update(Robot.kDefaultPeriod);
            simStateMotor.setRotorAcceleration(simMotor.getAngularAcceleration());
            simStateMotor.setRotorVelocity(simMotor.getAngularVelocity());
            simStateMotor.setRawRotorPosition(simMotor.getAngularPosition());
        }

        inputs.currentVelocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();

        inputs.gamePieceDetected = gamePieceDetector.get();
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}