package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

public class SimElevatorIO implements ElevatorIO {
    private static final Elevator.Constants CONSTANTS = new Elevator.Constants(
            Units.feetToMeters(0.0), Units.feetToMeters(8.0)
    );

    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double REDUCTION = 12.0;
    private static final double MASS = Units.lbsToKilograms(15.0);
    private static final double DRUM_RADIUS = Units.inchesToMeters(2.0);

    private final ElevatorSim sim = new ElevatorSim(
            MOTOR,
            REDUCTION,
            MASS,
            DRUM_RADIUS,
            CONSTANTS.minimumPosition(),
            CONSTANTS.maximumPosition(),
            true,
            CONSTANTS.minimumPosition());

    private final ProfiledPIDController controller = new ProfiledPIDController(
            100.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.feetToMeters(30.0),
            Units.feetToMeters(60.0)
    ));

    private double targetVoltage = Double.NaN;
    private double targetPosition = Double.NaN;

    @Override
    public Elevator.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double voltage = 0.0;
        if (Double.isFinite(targetPosition))
        {
            voltage = controller.calculate(sim.getPositionMeters(),targetPosition);
        }
        else if (Double.isFinite(targetVoltage))
        {
            voltage = targetVoltage;
        }
        sim.setInputVoltage(voltage);

        sim.update(Robot.kDefaultPeriod);

        inputs.currentAppliedVoltageMotor1 = sim.getInput().get(0,0);
        inputs.currentPositionMotor1 = sim.getPositionMeters();
        inputs.currentVelocityMotor1 = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setTargetVoltage(double voltage) {
        targetPosition = Double.NaN;
        targetVoltage = voltage;
    }

    @Override
    public void setTargetPosition(double position) {
        targetPosition = position;
        targetVoltage = Double.NaN;
    }

    @Override
    public void resetPosition() {
        sim.setState(0.0, sim.getVelocityMetersPerSecond());
    }
}
