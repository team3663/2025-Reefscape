package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
    private final Elevator elevator;
    private final Arm arm;
    private final double shoulderLength = 12.0;
    private final double buffer= 4.0;

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

//    private boolean armGoStraightDown(double elevatorCurrentPos, double shoulderCurrentPos, double elevatorTargetPos, double shoulderTargetPos) {
//        return (shoulderCurrentPos< shoulderTargetPos + Math.toRadians(180)) && (shoulderTargetPos < Math.toRadians(-90));
//    }
//
//    private double goingDownMinHeight(double shoulderAngle){
//        double targetElevatorPos= (shoulderLength*(Math.sin(shoulderAngle)+buffer));
//         return targetElevatorPos;
//    }
//    private double goingUpMinAngle(double elevatorHeight){
//        double bufferAngle= Math.cos((elevatorHeight / shoulderLength));
//        double allowableAngle = -(bufferAngle + Math.toRadians(90));
//        return allowableAngle;
//    }

    private double getAllowableHeight(double targetHeight, double armAngle){
        targetHeight= Math.max(targetHeight, shoulderLength * (Math.sin(armAngle) + buffer));
        return targetHeight;
    }

    private double getAllowableAngle(double targetAngle, double targetHeight){
        targetAngle= Math.max(targetAngle, ( - Math.cos((targetHeight/ shoulderLength) - Math.toRadians(90))));
        return targetAngle;
    }

/// make sure everything ends when your at the target




//    private boolean armNotCollide(double elevatorTargetPos, double armTargetPos) {
//        return armNotCollide(elevator.getPosition(), arm.getPosition(), elevatorTargetPos, armTargetPos);
//    }

    public Command stop() {
        return runOnce(
                () -> {
                    elevator.stop();
                    arm.stop();
                }
        );
    }


    public Command goToPositions(double elevatorPosition, double armPosition) {
        double height = getAllowableHeight(elevatorPosition, arm.getShoulderPosition());
        double angle = getAllowableAngle(armPosition,height);
        return Commands.parallel(
                arm.goToPositions(angle, angle),
                elevator.goToPosition(height)

        ).until(()-> (elevatorPosition==height) && (armPosition==angle));
    }


    public Command resetPositions() {
        return runOnce(
                () -> {
                    elevator.resetPosition();
//                    arm.resetPosition();
                }
        );
    }
}