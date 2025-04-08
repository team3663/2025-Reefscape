package frc.robot.subsystems.groundIntake;

import edu.wpi.first.math.util.Units;

public interface GroundIntakeIO {
    default GroundIntake.Constants getConstants() {
        return new GroundIntake.Constants(
                Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)
        );
    }

    default void updateInputs(GroundIntakeInputs inputs) {
    }

    default void setTargetPositionPivot(double position) {
    }

    default void setTargetVoltagePivot(double voltage) {
    }

    default void setTargetVoltageIntake(double voltage) {
    }

    default void resetPivotPosition(double position){

    }

    default void stopPivot() {
        setTargetVoltagePivot(0.0);
    }

    default void stopIntake() {
        setTargetVoltageIntake(0.0);
    }
}
