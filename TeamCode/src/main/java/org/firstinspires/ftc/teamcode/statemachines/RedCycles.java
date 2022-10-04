package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Toggles;
import org.firstinspires.ftc.teamcode.autonomous.FindCapstonePosition;
import org.firstinspires.ftc.teamcode.hardware.TrebuchetArm;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.mecanumdrive.MecanumAutonomous;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;

@Autonomous(name = "Red Cycles (State Machine)", group = "Autonomous")
public class RedCycles extends LinearOpMode {
    private FindCapstonePosition camera;
    private int region;
    private MecanumAutonomous drive;
    private TrebuchetArm trebuchet;
    private ElapsedTime timer, limit;
    private Toggles toggles;
    private Turret turret;
    private int parkTime;
    private int scoopTime;
    private boolean continueDrive = false;

    private RobotPosition[] positions = new RobotPosition[] {
            //hub
            new RobotPosition(new TwoDimensionalVector(2550, -2550), 0),
            //warehouse entry
            new RobotPosition(new TwoDimensionalVector(-100, -500), 0),
            //collection point
            new RobotPosition(new TwoDimensionalVector(-130, 3500), 0),
            //parking spot deep
            new RobotPosition(new TwoDimensionalVector(2600, 1200), 0)
    };

    enum DrivePosition {
        POSITION_START,
        POSITION_TOHUB,
        POSITION_HUB,
        POSITION_TOCOLLECTENTRYIN,
        POSITION_TOCOLLECT,
        POSITION_COLLECT,
        POSITION_TOCOLLECTENTRYOUT,
        POSITION_TODEPOTENTRY,
        POSITION_TODEPOT,
        POSITION_DEPOT
    }

    enum DuckState {
        FLYWHEEL_START,
        FLYWHEEL_ON,
        FLYWHEEL_OFF
    }

    enum TurretRotation {
        TURRET_START,
        TURRET_TORIGHTR,
        TURRET_RIGHTR,
        TURRET_TORIGHT,
        TURRET_RIGHT,
        TURRET_TOFRONT,
        TURRET_FRONT
    }

    enum TrebuchetPosition {
        TREBUCHET_START,
        TREBUCHET_UPR,
        TREBUCHET_TOREGIONHUB,
        TREBUCHET_REGIONHUB,
        TREBUCHET_UP1,
        TREBUCHET_TOHUB,
        TREBUCHET_HUB,
        TREBUCHET_UP2,
        TREBUCHET_TOLOWER,
        TREBUCHET_LOWER
    }

    enum ScoopState {
        SCOOP_START,
        SCOOP_IN,
        SCOOP_OUT,
        SCOOP_OFF
    }

    volatile DrivePosition drivePosition = DrivePosition.POSITION_START;
    volatile DuckState duckState = DuckState.FLYWHEEL_START;
    volatile TurretRotation turretRotation = TurretRotation.TURRET_START;
    volatile TrebuchetPosition trebuchetPosition = TrebuchetPosition.TREBUCHET_START;
    volatile ScoopState scoopState = ScoopState.SCOOP_START;

    @Override
    public void runOpMode() {
        drive = new MecanumAutonomous(hardwareMap, new RobotPosition(new TwoDimensionalVector(0, 0), 0), 50, 1000, 10, 180);
        toggles = new Toggles(hardwareMap);
        turret = new Turret(hardwareMap, "turret");
        camera = new FindCapstonePosition(hardwareMap, telemetry, this);
        trebuchet = new TrebuchetArm(hardwareMap, "trebuchet", "trebuchet2", /*"rotate",*/ 10000, 0, new int[] {0, 25, 50, 75, 100, 125, 150, 175, 200, 225}, 15, .015, 0.4, 0.1, 0.1, 0.1, 0.1);
        camera.init();
        waitForStart();
        region = camera.findRegion();
        sleep(500);
        double armTarget = -50;
        switch(region) {
            case 1:
                armTarget = -50;
                break;
            case 2:
                armTarget = -125;
                break;
            case 3:
                armTarget = -175;
                break;
        }
        timer = new ElapsedTime();
        limit = new ElapsedTime();
        //Set duckTime to however long it takes the flywheel to spin a duck off, respectively same for scoopTime
        parkTime = 5000;
        scoopTime = 1000;
        boolean marker1 = false;
        boolean marker2 = true;
        boolean markerR = true;
        boolean markerS = true;
        //toggles.flipIntake();
        //Implement switch case for region number here
        while (opModeIsActive()) {
            switch (drivePosition) {
                case POSITION_START:
                    limit.reset();
                    drivePosition = DrivePosition.POSITION_TOHUB;
                    break;
                case POSITION_TOHUB:
                    if (markerR) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_UPR;
                        markerR = false;
                    } else if (marker1) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_UP1;
                        marker1 = false;
                    }
                    if (drive.moveTo(positions[0])) {
                        drivePosition = DrivePosition.POSITION_HUB;
                        drive.stop();
                    }
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_HUB:
                    marker1 = true;
                    if (markerS) {
                        timer.reset();
                        markerS = false;
                    }
                    scoopState = ScoopState.SCOOP_OUT;
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_TOCOLLECTENTRYIN:
                    if (marker2) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_UP2;
                        marker2 = false;
                    }
                    if (drive.moveTo(positions[1])) {
                        drivePosition = DrivePosition.POSITION_TOCOLLECT;
                        drive.stop();
                    }
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_TOCOLLECT:
                    marker2 = true;
                    if (drive.moveTo(positions[2])) {
                        if (markerS) {
                            timer.reset();
                            markerS = false;
                        }
                        scoopState = ScoopState.SCOOP_IN;
                        drivePosition = DrivePosition.POSITION_COLLECT;
                        drive.stop();
                    }
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_COLLECT:
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_TOCOLLECTENTRYOUT:
                    if (drive.moveTo(positions[1])) {
                        drivePosition = DrivePosition.POSITION_TOHUB;
                        drive.stop();
                    }
                    if (limit.milliseconds() >= (30000 - parkTime)) {
                        drivePosition = DrivePosition.POSITION_TODEPOTENTRY;
                    }
                    break;
                case POSITION_TODEPOTENTRY:
                    if (drive.moveTo(positions[1])) {
                        drivePosition = DrivePosition.POSITION_TODEPOT;
                        drive.stop();
                    }
                    break;
                case POSITION_TODEPOT:
                    if (drive.moveTo(positions[3])) {
                        drivePosition = DrivePosition.POSITION_DEPOT;
                        drive.stop();
                    }
                    break;
                case POSITION_DEPOT:
                    stop();
                    break;
                default:
                    drivePosition = DrivePosition.POSITION_START;
            }
            switch (duckState) {
                case FLYWHEEL_START:
                    break;
                default:
                    duckState = DuckState.FLYWHEEL_START;
            }
            switch (turretRotation) {
                case TURRET_START:
                    telemetry.addData("turret start", null);
                    break;
                case TURRET_TORIGHTR:
                    if (turret.turnRight()) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOREGIONHUB;
                        turretRotation = TurretRotation.TURRET_RIGHTR;
                    }
                    break;
                case TURRET_RIGHTR:
                    break;
                case TURRET_TORIGHT:
                    if (turret.turnRight()) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOHUB;
                        turretRotation = TurretRotation.TURRET_RIGHT;
                    }
                    telemetry.addData("turret to right", null);
                    break;
                case TURRET_RIGHT:
                    telemetry.addData("turret right", null);
                    break;
                case TURRET_TOFRONT:
                    if (turret.turnForward()) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOLOWER;
                        turretRotation = TurretRotation.TURRET_FRONT;
                    }
                    telemetry.addData("turret to front", null);
                    break;
                case TURRET_FRONT:
                    telemetry.addData("turret front", null);
                    break;
                default:
                    turretRotation = TurretRotation.TURRET_START;
            }
            switch (trebuchetPosition) {
                case TREBUCHET_START:
                    telemetry.addData("trebuchet start", null);
                    break;
                case TREBUCHET_UPR:
                    if(trebuchet.moveToTarget(-175)) {
                        turretRotation = TurretRotation.TURRET_TORIGHTR;
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOREGIONHUB;
                    }
                    break;
                case TREBUCHET_TOREGIONHUB:
                    if(trebuchet.moveToTarget(armTarget)) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_REGIONHUB;
                    }
                    break;
                case TREBUCHET_REGIONHUB:
                    trebuchet.moveToTarget(armTarget);
                    break;
                case TREBUCHET_UP1:
                    if(trebuchet.moveToTarget(-175)) {
                        turretRotation = TurretRotation.TURRET_TORIGHT;
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOHUB;
                    }
                    break;
                case TREBUCHET_TOHUB:
                    if(trebuchet.moveToTarget(-175)) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_HUB;
                    }
                    telemetry.addData("trebuchet to hub", null);
                    break;
                case TREBUCHET_HUB:
                    trebuchet.moveToTarget(-175);
                    telemetry.addData("trebuchet hub", null);
                    break;
                case TREBUCHET_UP2:
                    if(trebuchet.moveToTarget(-175)) {
                        turretRotation = TurretRotation.TURRET_TOFRONT;
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOLOWER;
                    }
                    break;
                case TREBUCHET_TOLOWER:
                    if(trebuchet.moveToTarget(0)) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_LOWER;
                    }
                    telemetry.addData("trebuchet to lower", null);
                    break;
                case TREBUCHET_LOWER:
                    trebuchet.moveToTarget(0);
                    telemetry.addData("trebuchet lower", null);
                    break;
                default:
                    trebuchetPosition = TrebuchetPosition.TREBUCHET_START;
            }
            switch (scoopState) {
                case SCOOP_START:
                    telemetry.addData("scoop start", null);
                    break;
                case SCOOP_IN:
                    toggles.startIntake();
                    if (timer.milliseconds() >= scoopTime) {
                        drivePosition = DrivePosition.POSITION_TOCOLLECTENTRYOUT;
                        scoopState = ScoopState.SCOOP_OFF;
                        markerS = true;
                    }
                    break;
                case SCOOP_OUT:
                    //Intake already flipped from earlier
                    toggles.flipIntake();
                    if (timer.milliseconds() >= scoopTime) {
                        drivePosition = DrivePosition.POSITION_TOCOLLECTENTRYIN;
                        scoopState = ScoopState.SCOOP_OFF;
                        markerS = true;
                    }
                    telemetry.addData("scoop on", null);
                    break;
                case SCOOP_OFF:
                    toggles.stopIntake();
                    telemetry.addData("scoop off", null);
                    break;
                default:
                    scoopState = ScoopState.SCOOP_START;
            }
            telemetry.update();
        }
    }
}

//only one with fixed scoop timers
//scoop is running for too long, could reduce time
//turret bearing gets off course, how to fix
//have not tested park yet
//fix camera mount