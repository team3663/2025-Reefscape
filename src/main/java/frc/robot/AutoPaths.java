package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import choreo.auto.AutoFactory;
import frc.robot.subsystems.grabber.Grabber;

import java.sql.ParameterMetaData;


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
        this.commandFactory= commandFactory;

        this.arm = arm;
    }

    public AutoRoutine testAuto(){
        AutoRoutine routine = autoFactory.newRoutine("Test Auto");

        AutoTrajectory MoveForwardTraj = routine.trajectory("Move Forward");
        MoveForwardTraj.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_2));

        routine.active().onTrue(
                Commands.sequence(
                        MoveForwardTraj.resetOdometry(),
                        superStructure.resetPositions(),
                        MoveForwardTraj.cmd()
                )
        );

        MoveForwardTraj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));

        return routine;
    }

    public AutoRoutine facePlantG() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantG");

        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantG");
        facePlantGTraj.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4));

        routine.active().onTrue(
                Commands.sequence(
                        facePlantGTraj.resetOdometry(),
                        superStructure.resetPositions(),
                        Commands.waitSeconds(2).andThen(
                                Commands.parallel(
                                        facePlantGTraj.cmd()
                        )
                )
        ));
        facePlantGTraj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine facePlantH() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantH");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantH");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(Commands.parallel(
                                facePlantHTraj.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()))
                        )
                )
        );

        return routine;
    }

    public AutoRoutine twoCoralDC() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralDC");

        AutoTrajectory start = routine.trajectory("PStart-D");
        AutoTrajectory dwcs = routine.trajectory("D-WCS");
        AutoTrajectory wcsc = routine.trajectory("WCS-C");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        dwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsc.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        superStructure.resetPositions(),
                                start.cmd())
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        dwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcsc.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));

        return routine;
    }

    public AutoRoutine twoCoralFE() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralFE");

        AutoTrajectory start = routine.trajectory("PStart-F");
        AutoTrajectory fwcs = routine.trajectory("F-WCS");
        AutoTrajectory wcse = routine.trajectory("WCS-E");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        fwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcse.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine twoCoralIJ() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralIJ");

        AutoTrajectory start = routine.trajectory("LStart-I");
        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        ilwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsj.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        ilwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        lwcsj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));

        return routine;
    }

    public AutoRoutine twoCoralKL() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralKL");

        AutoTrajectory start = routine.trajectory("LStart-K");
        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        klwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsl.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        klwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        lwcsl.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));

        return routine;
    }

    public AutoRoutine behindTheBackAB() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackAB");

        AutoTrajectory start = routine.trajectory("PStart-A");
        AutoTrajectory adcs = routine.trajectory("A-DCS");
        AutoTrajectory dcsb = routine.trajectory("DCS-B");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        adcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        dcsb.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        adcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        dcsb.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine behindTheBackFlippedBA() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackFlippedBA");

        AutoTrajectory start = routine.trajectory("LStart-B");
        AutoTrajectory bldcs = routine.trajectory("B-LDCS");
        AutoTrajectory ldcsa = routine.trajectory("LDCS-A");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        bldcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        ldcsa.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        bldcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        ldcsa.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine threeCoralEDC() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralEDC");

        AutoTrajectory start = routine.trajectory("PStart-E");
        AutoTrajectory ewcs = routine.trajectory("E-WCS");
        AutoTrajectory wcsd = routine.trajectory("WCS-D");
        AutoTrajectory dwcs = routine.trajectory("D-WCS");
        AutoTrajectory wcsc = routine.trajectory("WCS-C");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        ewcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsd.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        dwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsc.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        ewcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcsd.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        dwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));dwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcsc.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine threeCoralGFE() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralGFE");

        AutoTrajectory start = routine.trajectory("FacePlantG");
        AutoTrajectory gwcs = routine.trajectory("G-WCS");
        AutoTrajectory wcsf = routine.trajectory("WCS-F");
        AutoTrajectory fwcs = routine.trajectory("F-WCS");
        AutoTrajectory wcse = routine.trajectory("WCS-E");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        gwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsf.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        fwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcse.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));


        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        gwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcsf.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine threeCoralJKL() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralJKL");

        AutoTrajectory start = routine.trajectory("LStart-J");
        AutoTrajectory jlwcs = routine.trajectory("J-LWCS");
        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        jlwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsk.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        klwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsl.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));


        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
                )
        );

        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        jlwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        lwcsk.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        klwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()));
        lwcsl.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine fourCoralFCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFCDE");

        AutoTrajectory start = routine.trajectory("POStart-F");
        AutoTrajectory fwcs = routine.trajectory("F-WCS");
        AutoTrajectory wcsc = routine.trajectory("WCS-C");
        AutoTrajectory cwcs = routine.trajectory("C-WCS");
        AutoTrajectory wcsd = routine.trajectory("WCS-D");
        AutoTrajectory dwcs = routine.trajectory("D-WCS");
        AutoTrajectory wcse = routine.trajectory("WCS-E");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        fwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsc.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        cwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsd.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        dwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcse.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));



        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        start.done().onTrue(Commands.parallel(fwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        fwcs.done().onTrue(Commands.parallel(wcsc.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        wcsc.done().onTrue(Commands.parallel(cwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        cwcs.done().onTrue(Commands.parallel(wcsd.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        wcsd.done().onTrue(Commands.parallel(dwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        dwcs.done().onTrue(Commands.parallel(wcse.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fourCoralFlippedILKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFlippedILKJ");

        AutoTrajectory start = routine.trajectory("LOStart-I");
        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
        AutoTrajectory llwcs = routine.trajectory("L-LWCS");
        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        ilwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsl.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        llwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsk.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        klwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        lwcsj.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));




        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        start.done().onTrue(Commands.parallel(ilwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ilwcs.done().onTrue(Commands.parallel(lwcsl.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        lwcsl.done().onTrue(Commands.parallel(llwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        llwcs.done().onTrue(Commands.parallel(lwcsk.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        lwcsk.done().onTrue(Commands.parallel(klwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        klwcs.done().onTrue(Commands.parallel(lwcsj.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fiveCoralFBCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFBCDE");

        AutoTrajectory start = routine.trajectory("POStart-F");
        AutoTrajectory fwcs = routine.trajectory("F-WCS");
        AutoTrajectory wcsb = routine.trajectory("WCS-B");
        AutoTrajectory bwcs = routine.trajectory("B-WCS");
        AutoTrajectory wcsc = routine.trajectory("WCS-C");
        AutoTrajectory cwcs = routine.trajectory("C-WCS");
        AutoTrajectory wcsd = routine.trajectory("WCS-D");
        AutoTrajectory dwcs = routine.trajectory("D-WCS");
        AutoTrajectory wcse = routine.trajectory("WCS-E");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        fwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsb.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        bwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsc.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        cwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcsd.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        dwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        wcse.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));



        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        start.done().onTrue(Commands.parallel(fwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        fwcs.done().onTrue(Commands.parallel(wcsb.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        wcsb.done().onTrue(Commands.parallel(bwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        bwcs.done().onTrue(Commands.parallel(wcsc.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        wcsc.done().onTrue(Commands.parallel(cwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        cwcs.done().onTrue(Commands.parallel(wcsd.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        wcsd.done().onTrue(Commands.parallel(dwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        dwcs.done().onTrue(Commands.parallel(wcse.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fiveCoralFlippedIALKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFlippedIALKJ");

        AutoTrajectory start = routine.trajectory("LOStart-I");
        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
        AutoTrajectory lwcsa = routine.trajectory("LWCS-A");
        AutoTrajectory alwcs = routine.trajectory("A-LWCS");
        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
        AutoTrajectory llwcs = routine.trajectory("L-LWCS");
        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
        start.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        ilwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsa.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        alwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        lwcsl.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        llwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        lwcsk.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));
        klwcs.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3));
        lwcsj.atTimeBeforeEnd(0.25).onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION));




        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        start.done().onTrue(Commands.parallel(ilwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ilwcs.done().onTrue(Commands.parallel(lwcsa.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        lwcsa.done().onTrue(Commands.parallel(alwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        alwcs.done().onTrue(Commands.parallel(lwcsl.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        lwcsl.done().onTrue(Commands.parallel(llwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        llwcs.done().onTrue(Commands.parallel(lwcsk.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        lwcsk.done().onTrue(Commands.parallel(klwcs.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        klwcs.done().onTrue(Commands.parallel(lwcsj.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

}
