package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import choreo.auto.AutoFactory;
import frc.robot.subsystems.grabber.Grabber;


public class AutoPaths {
    private final Drivetrain drivetrain;
    private final Grabber grabber;
    private final Arm arm;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final CommandFactory commandFactory;


    public AutoPaths(
            Drivetrain drivetrain, Grabber grabber,
            SuperStructure superStructure, AutoFactory autoFactory, Arm arm, CommandFactory commandFactory) {
        this.drivetrain = drivetrain;
        this.grabber = grabber;
        this.superStructure = superStructure;
        this.autoFactory = autoFactory;
        this.commandFactory = commandFactory;

        this.arm = arm;
    }

    public AutoRoutine facePlantD1() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D1");

        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantD1");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantGTraj.resetOdometry(),
                        superStructure.zero(),
                        facePlantGTraj.cmd()

                ));
        facePlantGTraj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine facePlantD2() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D2");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantD2");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        superStructure.zero(),
                        facePlantHTraj.cmd()

                ));
        facePlantHTraj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine twoCoralB2B1() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:B2-B1");

        AutoTrajectory start = routine.trajectory("RStart-B2");
        AutoTrajectory dwcs = routine.trajectory("B2-RS");
        AutoTrajectory wcsc = routine.trajectory("RS-B1");
        AutoTrajectory cwcs = routine.trajectory("B1-RS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                ));

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(dwcs.cmd()));
        dwcs.done().onTrue(commandFactory.grabCoral().andThen(wcsc.cmd()));
        wcsc.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(cwcs.cmd()));

        return routine;
    }

    public AutoRoutine twoCoralF1F2() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:F1-F2");

        AutoTrajectory start = routine.trajectory("LStart-F1");
        AutoTrajectory klwcs = routine.trajectory("F1-LS");
        AutoTrajectory lwcsl = routine.trajectory("LS-F2");
        AutoTrajectory llwcs = routine.trajectory("F2-LS");

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                )
        );

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(klwcs.cmd()));
        klwcs.done().onTrue(commandFactory.grabCoral().andThen(lwcsl.cmd()));
        lwcsl.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(llwcs.cmd()));
        return routine;
    }
}