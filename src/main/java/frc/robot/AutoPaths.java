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

    public AutoRoutine facePlantG() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantG");

        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantG");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantGTraj.resetOdometry(),
                        superStructure.zero(),
                        facePlantGTraj.cmd()

                ));
        facePlantGTraj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine facePlantH() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantH");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantH");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        superStructure.zero(),
                        facePlantHTraj.cmd()

                ));
        facePlantHTraj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine twoCoralDC() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralDC");

        AutoTrajectory start = routine.trajectory("PStart-D");
        AutoTrajectory dwcs = routine.trajectory("D-WCS");
        AutoTrajectory wcsc = routine.trajectory("WCS-C");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                ));

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(dwcs.cmd()));
        dwcs.done().onTrue(commandFactory.grabCoral().andThen(wcsc.cmd()));
        wcsc.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));

        return routine;
    }

    public AutoRoutine twoCoralKL() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralKL");

        AutoTrajectory start = routine.trajectory("LStart-K");
        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        Commands.parallel(
                                start.cmd(), superStructure.zero())
                )
        );

        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()).andThen(klwcs.cmd()));
        klwcs.done().onTrue(commandFactory.grabCoral().andThen(lwcsl.cmd()));
        lwcsl.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

//    public AutoRoutine twoCoralFE() {
//        AutoRoutine routine = autoFactory.newRoutine("TwoCoralFE");
//
//        AutoTrajectory start = routine.trajectory("PStart-F");
//        AutoTrajectory fwcs = routine.trajectory("F-WCS");
//        AutoTrajectory wcse = routine.trajectory("WCS-E");
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(fwcs.cmd()));
//        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcse.cmd()));
//        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }

//    public AutoRoutine twoCoralIJ() {
//        AutoRoutine routine = autoFactory.newRoutine("TwoCoralIJ");
//
//        AutoTrajectory start = routine.trajectory("LStart-I");
//        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
//        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(ilwcs.cmd()));
//        ilwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsj.cmd()));
//        lwcsj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//
//        return routine;
//    }

//    public AutoRoutine behindTheBackAB() {
//        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackAB");
//
//        AutoTrajectory start = routine.trajectory("PStart-A");
//        AutoTrajectory adcs = routine.trajectory("A-DCS");
//        AutoTrajectory dcsb = routine.trajectory("DCS-B");
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(adcs.cmd()));
//        adcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(dcsb.cmd()));
//        dcsb.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }
//
//    public AutoRoutine behindTheBackFlippedBA() {
//        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackFlippedBA");
//
//        AutoTrajectory start = routine.trajectory("LStart-B");
//        AutoTrajectory bldcs = routine.trajectory("B-LDCS");
//        AutoTrajectory ldcsa = routine.trajectory("LDCS-A");
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(bldcs.cmd()));
//        bldcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(ldcsa.cmd()));
//        ldcsa.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }
//
//    public AutoRoutine threeCoralEDC() {
//        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralEDC");
//
//        AutoTrajectory start = routine.trajectory("PStart-E");
//        AutoTrajectory ewcs = routine.trajectory("E-WCS");
//        AutoTrajectory wcsd = routine.trajectory("WCS-D");
//        AutoTrajectory dwcs = routine.trajectory("D-WCS");
//        AutoTrajectory wcsc = routine.trajectory("WCS-C");
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.zero())
//                ));
//
//        start.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3).andThen(Commands.waitSeconds(0.25)).andThen(commandFactory.placeCoral()).andThen(ewcs.cmd()));
//        ewcs.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION).andThen(commandFactory.grabCoral()).andThen(wcsd.cmd()));
//        wcsd.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3).andThen(Commands.waitSeconds(0.25)).andThen(commandFactory.placeCoral()).andThen(dwcs.cmd()));
//        dwcs.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_STATION).andThen(commandFactory.grabCoral()).andThen(wcsc.cmd()));
//        wcsc.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_3).andThen(Commands.waitSeconds(0.25)).andThen(commandFactory.placeCoral()));
//
//        return routine;
//    }
//
//    public AutoRoutine threeCoralGFE() {
//        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralGFE");
//
//        AutoTrajectory start = routine.trajectory("FacePlantG");
//        AutoTrajectory gwcs = routine.trajectory("G-WCS");
//        AutoTrajectory wcsf = routine.trajectory("WCS-F");
//        AutoTrajectory fwcs = routine.trajectory("F-WCS");
//        AutoTrajectory wcse = routine.trajectory("WCS-E");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(gwcs.cmd()));
//        gwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsf.cmd()));
//        wcsf.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(fwcs.cmd()));
//        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcse.cmd()));
//        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }
//
//    public AutoRoutine threeCoralJKL() {
//        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralJKL");
//
//        AutoTrajectory start = routine.trajectory("LStart-J");
//        AutoTrajectory jlwcs = routine.trajectory("J-LWCS");
//        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
//        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
//        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)).andThen(commandFactory.placeCoral())
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(jlwcs.cmd()));
//        jlwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsk.cmd()));
//        lwcsk.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(klwcs.cmd()));
//        klwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsl.cmd()));
//        lwcsl.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }
//
//    public AutoRoutine fourCoralFCDE() {
//        AutoRoutine routine = autoFactory.newRoutine("FourCoralFCDE");
//
//        AutoTrajectory start = routine.trajectory("POStart-F");
//        AutoTrajectory fwcs = routine.trajectory("F-WCS");
//        AutoTrajectory wcsc = routine.trajectory("WCS-C");
//        AutoTrajectory cwcs = routine.trajectory("C-WCS");
//        AutoTrajectory wcsd = routine.trajectory("WCS-D");
//        AutoTrajectory dwcs = routine.trajectory("D-WCS");
//        AutoTrajectory wcse = routine.trajectory("WCS-E");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(fwcs.cmd()));
//        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsc.cmd()));
//        wcsc.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(cwcs.cmd()));
//        cwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsd.cmd()));
//        wcsd.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(dwcs.cmd()));
//        dwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcse.cmd()));
//        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//
//        return routine;
//    }
//
//    public AutoRoutine fourCoralFlippedILKJ() {
//        AutoRoutine routine = autoFactory.newRoutine("FourCoralFlippedILKJ");
//
//        AutoTrajectory start = routine.trajectory("LOStart-I");
//        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
//        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
//        AutoTrajectory llwcs = routine.trajectory("L-LWCS");
//        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
//        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
//        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(ilwcs.cmd()));
//        ilwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsl.cmd()));
//        lwcsl.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(llwcs.cmd()));
//        llwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsk.cmd()));
//        lwcsk.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(klwcs.cmd()));
//        klwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsj.cmd()));
//        lwcsj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//
//        return routine;
//    }
//
//    public AutoRoutine fiveCoralFBCDE() {
//        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFBCDE");
//
//        AutoTrajectory start = routine.trajectory("POStart-F");
//        AutoTrajectory fwcs = routine.trajectory("F-WCS");
//        AutoTrajectory wcsb = routine.trajectory("WCS-B");
//        AutoTrajectory bwcs = routine.trajectory("B-WCS");
//        AutoTrajectory wcsc = routine.trajectory("WCS-C");
//        AutoTrajectory cwcs = routine.trajectory("C-WCS");
//        AutoTrajectory wcsd = routine.trajectory("WCS-D");
//        AutoTrajectory dwcs = routine.trajectory("D-WCS");
//        AutoTrajectory wcse = routine.trajectory("WCS-E");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(fwcs.cmd()));
//        fwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsb.cmd()));
//        wcsb.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(bwcs.cmd()));
//        bwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsc.cmd()));
//        wcsc.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(cwcs.cmd()));
//        cwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcsd.cmd()));
//        wcsd.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(dwcs.cmd()));
//        dwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(wcse.cmd()));
//        wcse.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//        return routine;
//    }
//
//    public AutoRoutine fiveCoralFlippedIALKJ() {
//        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFlippedIALKJ");
//
//        AutoTrajectory start = routine.trajectory("LOStart-I");
//        AutoTrajectory ilwcs = routine.trajectory("I-LWCS");
//        AutoTrajectory lwcsa = routine.trajectory("LWCS-A");
//        AutoTrajectory alwcs = routine.trajectory("A-LWCS");
//        AutoTrajectory lwcsl = routine.trajectory("LWCS-L");
//        AutoTrajectory llwcs = routine.trajectory("L-LWCS");
//        AutoTrajectory lwcsk = routine.trajectory("LWCS-K");
//        AutoTrajectory klwcs = routine.trajectory("K-LWCS");
//        AutoTrajectory lwcsj = routine.trajectory("LWCS-J");
//
//
//        routine.active().onTrue(
//                Commands.sequence(
//                        start.resetOdometry(),
//                        Commands.parallel(
//                                start.cmd(),
//                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
//                )
//        );
//
//        start.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(ilwcs.cmd()));
//        ilwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsa.cmd()));
//        lwcsa.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(alwcs.cmd()));
//        alwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsl.cmd()));
//        lwcsl.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(llwcs.cmd()));
//        llwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsk.cmd()));
//        lwcsk.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()).andThen(klwcs.cmd()));
//        klwcs.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.grabCoral()).andThen(lwcsj.cmd()));
//        lwcsj.done().onTrue(Commands.waitUntil(superStructure::atTargetPositions).andThen(commandFactory.placeCoral()));
//
//        return routine;
//    }

}
