package frc.robot.subsystems.climber;

public interface ClimberIO {
    default void updateInputs(ClimberInputs inputs){}
    default void setTargetPosition(double position){}
    default void setTargetVoltage(double voltage){}
    default void currentAppliedVoltage(double voltage){}
    default void resetPosition(){setTargetPosition(0.0);}
    default void stop(){currentAppliedVoltage(0.0);}
}
