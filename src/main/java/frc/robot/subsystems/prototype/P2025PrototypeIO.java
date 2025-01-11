package frc.robot.subsystems.prototype;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class P2025PrototypeIO implements PrototypeIO {
    private final TalonFX motor1;
    private final TalonFX motor2;

    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2),
                    0.001, 1.0),
            DCMotor.getKrakenX60(2).withReduction(1.0)
    );;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withSlot(1);

    public P2025PrototypeIO(TalonFX motor1, TalonFX motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;

        // configuring the motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        //config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //Slot 0 PID for velocity
        config.Slot0.kP = 0.05;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.115;

        //Slot 1 PID/Motion Magic for position
        config.Slot1.kP = 0.5;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        config.Slot1.kA = 0.01;
        config.Slot1.kV = 0.115;
        config.MotionMagic.MotionMagicAcceleration = (1000.0 / 60.0);
        config.MotionMagic.MotionMagicCruiseVelocity = (5500.0 / 60.0);

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(PrototypeInputs inputs) {
        // Sims for Motor 1
        var simStateMotor1 = motor1.getSimState();
        sim.setInputVoltage(simStateMotor1.getMotorVoltage());
        //Updates the sim information every 20 ms
        sim.update(Robot.kDefaultPeriod);
        simStateMotor1.setRotorAcceleration(sim.getAngularAcceleration());
        simStateMotor1.setRotorVelocity(sim.getAngularVelocity());
        simStateMotor1.setRawRotorPosition(sim.getAngularPosition());

        // Sims for Motor 2
        var simStateMotor2 = motor2.getSimState();
        sim.setInputVoltage(simStateMotor1.getMotorVoltage());
        //Updates the sim information every 20 ms
        sim.update(Robot.kDefaultPeriod);
        simStateMotor2.setRotorAcceleration(sim.getAngularAcceleration());
        simStateMotor2.setRotorVelocity(sim.getAngularVelocity());
        simStateMotor2.setRawRotorPosition(sim.getAngularPosition());


        // updating the Prototype inputs for motor 1
        inputs.currentVelocityMotor1 = Units.rotationsToRadians(motor1.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor1 = motor1.getMotorVoltage().getValueAsDouble();
        inputs.motorTemperatureMotor1 = motor1.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor1 = motor1.getSupplyCurrent().getValueAsDouble();
        inputs.currentRotationsMotor1 = Units.rotationsToRadians(motor1.getPosition().getValueAsDouble());

        // updating the Prototype inputs for motor 2
        inputs.currentVelocityMotor2 = Units.rotationsToRadians(motor2.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor2 = motor2.getMotorVoltage().getValueAsDouble();
        inputs.motorTemperatureMotor2 = motor2.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor2 = motor2.getSupplyCurrent().getValueAsDouble();
        inputs.currentRotationsMotor2 = Units.rotationsToRadians(motor2.getPosition().getValueAsDouble());
    }

    @Override
    public void resetRotationsMotor1(){
        motor1.setPosition(0);
    }

    @Override
    public void stopMotor1() {
        motor1.setControl(stopRequest);
    }

    @Override
    public void setTargetRotationsMotor1(double radians){
        motor1.setControl(positionRequest.withPosition(Units.radiansToRotations(radians)));
    }

    @Override
    public void setTargetVelocityMotor1(double velocity) {
        motor1.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }

    @Override
    public void setTargetVoltageMotor1(double voltage) {
        motor1.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void resetRotationsMotor2(){
        motor2.setPosition(0);
    }

    @Override
    public void stopMotor2() {
        motor2.setControl(stopRequest);
    }

    @Override
    public void stopMotors(){
        motor1.setControl(stopRequest);
        motor2.setControl(stopRequest);
    }

    @Override
    public void setTargetRotationsMotor2(double radians){
        motor2.setControl(positionRequest.withPosition(Units.radiansToRotations(radians)));
    }

    @Override
    public void setTargetVelocityMotor2(double velocity) {
        motor2.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }

    @Override
    public void setTargetVoltageMotor2(double voltage) {
        motor2.setControl(voltageRequest.withOutput(voltage));
    }
}
