package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

//TODO add second CANdi
public class C2025ClimberIO implements ClimberIO{
    private final TalonFX motor;
    private final CANdi beamBrake1;
    private final CANdi beamBrake2;
    private final NeutralOut stopRequest = new NeutralOut();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

    public C2025ClimberIO(TalonFX motor, CANdi beamBrake1, CANdi beamBrake2) {
        this.motor = motor;
        this.beamBrake1 = beamBrake1;
        this.beamBrake2 = beamBrake2;

        TalonFXConfiguration config = new TalonFXConfiguration();
        //PID for position
        config.Slot0.kV = 0.115;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0/60.0;
        config.MotionMagic.MotionMagicAcceleration = 2500.0/60.0;

        motor.getConfigurator().apply(config);

        CANdiConfiguration CANdiConfig = new CANdiConfiguration();
        beamBrake1.getConfigurator().apply(CANdiConfig);
        beamBrake2.getConfigurator().apply(CANdiConfig);
    }
    @Override
    public void updateInputs(ClimberInputs inputs){
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentPosition = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();

        inputs.gamePieceDetected1 = beamBrake1.getS1State().getValue() == S1StateValue.High;
        inputs.gamePieceDetected2 = beamBrake2.getS1State().getValue() == S1StateValue.High;

        // SIM STATES
        if (Robot.isSimulation()) {
            var simStateMotor = motor.getSimState();
            sim.setInputVoltage(simStateMotor.getMotorVoltage());
            // Updates sim every 20 milliseconds
            sim.update(Robot.kDefaultPeriod);
            simStateMotor.setRotorAcceleration(sim.getAngularAcceleration());
            simStateMotor.setRotorVelocity(sim.getAngularVelocity());
            simStateMotor.setRawRotorPosition(sim.getAngularPosition());
        }
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
        motor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

}
