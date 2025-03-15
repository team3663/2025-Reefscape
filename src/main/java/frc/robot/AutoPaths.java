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

        AutoTrajectory facePlantD1Traj = routine.trajectory("FacePlantD1");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantD1Traj.resetOdometry(),
                        superStructure.zero(),
                        facePlantD1Traj.cmd()

                ));
        facePlantD1Traj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine facePlantD2() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D2");

        AutoTrajectory facePlantD2Traj = routine.trajectory("FacePlantD2");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantD2Traj.resetOdometry(),
                        superStructure.zero(),
                        facePlantD2Traj.cmd()

                ));
        facePlantD2Traj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine twoCoralB2B1() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:B2-B1");

        AutoTrajectory start = routine.trajectory("RStart-B2");
        AutoTrajectory b2rs = routine.trajectory("B2-RS");
        AutoTrajectory rsb1 = routine.trajectory("RS-B1");
        AutoTrajectory b1rs = routine.trajectory("B1-RS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                ));

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(b2rs.cmd()));
        b2rs.done().onTrue(commandFactory.grabCoral().andThen(rsb1.cmd()));
        rsb1.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(b1rs.cmd()));

        return routine;
    }

    public AutoRoutine twoCoralF1F2() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:F1-F2");

        AutoTrajectory start = routine.trajectory("LStart-F1");
        AutoTrajectory f1ls = routine.trajectory("F1-LS");
        AutoTrajectory lsf2 = routine.trajectory("LS-F2");
        AutoTrajectory f2ls = routine.trajectory("F2-LS");

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                )
        );

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(f1ls.cmd()));
        f1ls.done().onTrue(commandFactory.grabCoral().andThen(lsf2.cmd()));
        lsf2.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(f2ls.cmd()));
        return routine;
    }
}