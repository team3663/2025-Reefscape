package frc.robot.subsystem.prototype;

public interface PrototypeIO {
    default void updateInputs(PrototypeInputs inputs) {
    }
    default void stop() {
        setTargetVoltage1(0.0);
        setTargetVoltage2(0.0);
    }
    default void stop1(){
        setTargetVoltage1(0.0);}
    default void setTargetVoltage1(double voltage){
    }
    default void setTargetVelocity1(double velocity){
    }
    default void setTargetPosition1(double position){
    }
    default void resetPosition1(){}
    default void stop2(){
        setTargetVoltage2(0.0);}
    default void setTargetVoltage2(double voltage){
    }
    default void setTargetVelocity2(double velocity){
    }
    default void setTargetPosition2(double position){
    }
    default void resetPosition2(){}
}
