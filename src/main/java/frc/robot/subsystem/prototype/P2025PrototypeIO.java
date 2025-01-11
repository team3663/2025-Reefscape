package frc.robot.subsystem.prototype;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static com.ctre.phoenix6.signals.InvertedValue.*;

public class P2025PrototypeIO implements PrototypeIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0)
    );
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(0);


    public P2025PrototypeIO(TalonFX motor1, TalonFX motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;



        TalonFXConfiguration config = new TalonFXConfiguration();
        //config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Slot 0 for velocity
        config.Slot0.kV = 0.115;
        config.Slot0.kP = 0.05;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;


        // motion magic for position
        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;
        config.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

    }


    @Override
    public void resetPosition1(){motor1.setPosition(0); }
    @Override
    public void stop1(){motor1.setControl(stopRequest);}
    @Override
    public void setTargetVelocity1(double velocity) {
        motor1.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }
    @Override
    public void setTargetVoltage1(double voltage) {motor1.setControl(voltageRequest.withOutput(voltage));}
    @Override
    public void setTargetPosition1(double position) {
        motor1.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void updateInputs(PrototypeInputs inputs) {
        inputs.currentVelocity2 = Units.rotationsToRadians(motor2.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltage2 = motor2.getMotorVoltage().getValueAsDouble();
        inputs.currentPosition2 = Units.rotationsToRadians(motor2.getPosition().getValueAsDouble());
        inputs.motorTemperature2 = motor2.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw2 = motor2.getSupplyCurrent().getValueAsDouble();
        inputs.currentVelocity1 = Units.rotationsToRadians(motor1.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltage1 = motor1.getMotorVoltage().getValueAsDouble();
        inputs.currentPosition1 = Units.rotationsToRadians(motor1.getPosition().getValueAsDouble());
        inputs.motorTemperature1 = motor1.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw1 = motor1.getSupplyCurrent().getValueAsDouble();

        //TODO add sim states here
        var simStateMotor1 = motor1.getSimState();
        var simStateMotor2 = motor2.getSimState();
        sim.setInputVoltage(simStateMotor1.getMotorVoltage());
        sim.setInputVoltage(simStateMotor2.getMotorVoltage());
        //Updates sim every 20 milliseconds
        sim.update(Robot.kDefaultPeriod);

        simStateMotor1.setRotorAcceleration(sim.getAngularAcceleration());
        simStateMotor1.setRotorVelocity(sim.getAngularVelocity());
        simStateMotor1.setRawRotorPosition(sim.getAngularPosition());

        simStateMotor2.setRotorAcceleration(sim.getAngularAcceleration());
        simStateMotor2.setRotorVelocity(sim.getAngularVelocity());
        simStateMotor2.setRawRotorPosition(sim.getAngularPosition());
    }
    @Override
    public void resetPosition2(){motor2.setPosition(0); }
    @Override
    public void stop2(){motor2.setControl(stopRequest);}
    @Override
    public void setTargetVelocity2(double velocity) {
        motor2.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }
    @Override
    public void setTargetVoltage2(double voltage) {motor2.setControl(voltageRequest.withOutput(voltage));}
    @Override
    public void setTargetPosition2(double position) {
        motor2.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

}