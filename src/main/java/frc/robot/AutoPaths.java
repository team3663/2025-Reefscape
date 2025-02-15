package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoPaths {
    private final Drivetrain drivetrain;
    private final SuperStructure superStructure;
    private final choreo.auto.AutoFactory autoFactory;

    public AutoPaths(
            Drivetrain drivetrain,
            SuperStructure superStructure, choreo.auto.AutoFactory autoFactory) {
        this.drivetrain =drivetrain;
        this.superStructure =superStructure;
        this.autoFactory =autoFactory;
    }

    public AutoRoutine facePlantG() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantG");

        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantG");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantGTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(
                                Commands.parallel(
                                        facePlantGTraj.cmd(),
                                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                        )
                )
        );
        return routine;
    }

    public AutoRoutine facePlantH() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantH");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantH");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(Commands.parallel(
                                facePlantHTraj.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                        )
                )
        );

        return routine;
    }

    public AutoRoutine twoCoralDC() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralDC");

        AutoTrajectory Start = routine.trajectory("PStart-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine twoCoralFE() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralFE");

        AutoTrajectory Start = routine.trajectory("PStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine twoCoralIJ() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralIJ");

        AutoTrajectory Start = routine.trajectory("LStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine twoCoralKL() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralKL");

        AutoTrajectory Start = routine.trajectory("LStart-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine behindTheBackAB() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackAB");

        AutoTrajectory Start = routine.trajectory("PStart-A");
        AutoTrajectory ADCS = routine.trajectory("A-DCS");
        AutoTrajectory DCSB = routine.trajectory("DCS-B");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(ADCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ADCS.done().onTrue(Commands.parallel(DCSB.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine behindTheBackFlippedBA() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackFlippedBA");

        AutoTrajectory Start = routine.trajectory("LStart-B");
        AutoTrajectory BLDCS = routine.trajectory("B-LDCS");
        AutoTrajectory LDCSA = routine.trajectory("LDCS-A");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(BLDCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        BLDCS.done().onTrue(Commands.parallel(LDCSA.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine threeCoralEDC() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralEDC");

        AutoTrajectory Start = routine.trajectory("PStart-E");
        AutoTrajectory EWCS = routine.trajectory("E-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(EWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        EWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine threeCoralGFE() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralGFE");

        AutoTrajectory Start = routine.trajectory("FacePlantG");
        AutoTrajectory GWCS = routine.trajectory("G-WCS");
        AutoTrajectory WCSF = routine.trajectory("WCS-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(GWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        GWCS.done().onTrue(Commands.parallel(WCSF.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSF.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine threeCoralJKL() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralJKL");

        AutoTrajectory Start = routine.trajectory("LStart-J");
        AutoTrajectory JLWCS = routine.trajectory("J-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(JLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        JLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fourCoralFCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFCDE");

        AutoTrajectory Start = routine.trajectory("POStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");
        AutoTrajectory CWCS = routine.trajectory("C-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSC.done().onTrue(Commands.parallel(CWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        CWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fourCoralFlippedILKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFlippedILKJ");

        AutoTrajectory Start = routine.trajectory("LOStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSL.done().onTrue(Commands.parallel(LLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        LLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fiveCoralFBCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFBCDE");

        AutoTrajectory Start = routine.trajectory("POStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSB = routine.trajectory("WCS-B");
        AutoTrajectory BWCS = routine.trajectory("B-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");
        AutoTrajectory CWCS = routine.trajectory("C-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSB.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSB.done().onTrue(Commands.parallel(BWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        BWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSC.done().onTrue(Commands.parallel(CWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        CWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    public AutoRoutine fiveCoralFlippedIALKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFlippedIALKJ");

        AutoTrajectory Start = routine.trajectory("LOStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSA = routine.trajectory("LWCS-A");
        AutoTrajectory ALWCS = routine.trajectory("A-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");
        
        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSA.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSA.done().onTrue(Commands.parallel(ALWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ALWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSL.done().onTrue(Commands.parallel(LLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        LLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

}
