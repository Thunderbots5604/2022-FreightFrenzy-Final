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

@Autonomous(name = "Red Full Run (State Machine)", group = "Autonomous")
public class RedFull extends LinearOpMode {
    private FindCapstonePosition camera;
    private int region;
    private MecanumAutonomous drive;
    private TrebuchetArm trebuchet;
    private ElapsedTime timer;
    private Toggles toggles;
    private Turret turret;
    private int duckTime;
    private int scoopTime;
    private boolean continueDrive = false;

    private RobotPosition[] positions = new RobotPosition[] {
            //start position
            new RobotPosition(new TwoDimensionalVector(200, 0), 0),
            //carousel
            new RobotPosition(new TwoDimensionalVector(200, -2100), 0),
            //hub
            new RobotPosition(new TwoDimensionalVector(3800, 1000), 0),
            //
            new RobotPosition(new TwoDimensionalVector(50, 6500), 0),
            //
            new RobotPosition(new TwoDimensionalVector(50, 8000), 0)
    };

    enum DrivePosition {
        POSITION_START,
        POSITION_TOCAROUSEL,
        POSITION_CAROUSEL,
        POSITION_TOHUB,
        POSITION_HUB,
        POSITION_TODEPOTENTRY,
        POSITION_TODEPOT,
        POSITION_DEPOT
    }

    enum DuckState {
        FLYWHEEL_START,
        FLYWHEEL_TOON,
        FLYWHEEL_ON,
        FLYWHEEL_TOOFF,
        FLYWHEEL_OFF
    }

    enum TurretRotation {
        TURRET_START,
        TURRET_TORIGHT,
        TURRET_RIGHT,
        TURRET_TOFRONT,
        TURRET_FRONT
    }

    enum TrebuchetPosition {
        TREBUCHET_START,
        TREBUCHET_UP1,
        TREBUCHET_TOHUB,
        TREBUCHET_HUB,
        TREBUCHET_UP2,
        TREBUCHET_TOLOWER,
        TREBUCHET_LOWER
    }

    enum ScoopState {
        SCOOP_START,
        SCOOP_ON,
        SCOOP_OFF
    }

    volatile DrivePosition drivePosition = DrivePosition.POSITION_START;
    volatile DuckState duckState = DuckState.FLYWHEEL_START;
    volatile TurretRotation turretRotation = TurretRotation.TURRET_START;
    volatile TrebuchetPosition trebuchetPosition = TrebuchetPosition.TREBUCHET_START;
    volatile ScoopState scoopState = ScoopState.SCOOP_START;

    private boolean[] complete = {false, false, false, false, false};

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
        //Set duckTime to however long it takes the flywheel to spin a duck off, respectively same for scoopTime
        duckTime = 3000;
        scoopTime = 1000;
        boolean marker1 = true;
        boolean marker2 = true;
        boolean markerS = true;
        //toggles.flipIntake();
        //Implement switch case for region number here
        while (opModeIsActive()) {
            switch (drivePosition) {
                case POSITION_START:
                    if (drive.moveTo(positions[0])) {
                        drivePosition = DrivePosition.POSITION_TOCAROUSEL;
                        drive.stop();
                    }
                    telemetry.addData("start position", null);
                    telemetry.update();
                    break;
                case POSITION_TOCAROUSEL:
                    if (drive.moveTo(positions[1])) {
                        if (markerS) {
                            timer.reset();
                            markerS = false;
                        }
                        duckState = DuckState.FLYWHEEL_TOON;
                        drivePosition = DrivePosition.POSITION_CAROUSEL;
                        drive.stop();
                    }
                    telemetry.addData("to carousel", null);
                    telemetry.update();
                    break;
                case POSITION_CAROUSEL:
                    if (duckState == DuckState.FLYWHEEL_OFF) {
                        drivePosition = DrivePosition.POSITION_TOHUB;
                    }
                    telemetry.addData("carousel", null);
                    telemetry.update();
                    break;
                case POSITION_TOHUB:
                    if (marker1) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_UP1;
                        marker1 = false;
                    }
                    if (drive.moveTo(positions[2])) {
                        drivePosition = DrivePosition.POSITION_HUB;
                        drive.stop();
                    }
                    telemetry.addData("to hub", null);
                    break;
                case POSITION_HUB:
                    if (markerS) {
                        timer.reset();
                        markerS = false;
                    }
                    scoopState = ScoopState.SCOOP_ON;
                    telemetry.addData("hub", null);
                    break;
                case POSITION_TODEPOTENTRY:
                    if (marker2) {
                        trebuchetPosition = TrebuchetPosition.TREBUCHET_TOLOWER;
                        marker2 = false;
                    }
                    if (drive.moveTo(positions[3])) {
                        drivePosition = DrivePosition.POSITION_TODEPOT;
                        drive.stop();
                    }
                    telemetry.addData("to depot entry", null);
                    break;
                case POSITION_TODEPOT:
                    if(drive.moveTo(positions[4])) {
                        drivePosition = DrivePosition.POSITION_DEPOT;
                        drive.stop();
                    }
                    telemetry.addData("to depot", null);
                    break;
                case POSITION_DEPOT:
                    complete[0] = true;
                    telemetry.addData("depot", null);
                    break;
                default:
                    drivePosition = DrivePosition.POSITION_START;
            }
        }
        switch (duckState) {
            case FLYWHEEL_START:
                telemetry.addData("flywheel start", null);
                break;
            case FLYWHEEL_TOON:
                toggles.startFlywheel(true);
                duckState = DuckState.FLYWHEEL_ON;
                break;
            case FLYWHEEL_ON:
                if (timer.milliseconds() >= duckTime) {
                    duckState = DuckState.FLYWHEEL_OFF;
                    markerS = true;
                }
                telemetry.addData("flywheel on", null);
                break;
            case FLYWHEEL_TOOFF:
                toggles.stopFlywheel();
                duckState = DuckState.FLYWHEEL_OFF;
                break;
            case FLYWHEEL_OFF:
                complete[1] = true;
                telemetry.addData("flywheel off", null);
                break;
            default:
                duckState = DuckState.FLYWHEEL_START;
        }
        switch (turretRotation) {
            case TURRET_START:
                telemetry.addData("turret start", null);
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
                complete[2] = true;
                telemetry.addData("turret front", null);
                break;
            default:
                turretRotation = TurretRotation.TURRET_START;
        }
        switch (trebuchetPosition) {
            case TREBUCHET_START:
                telemetry.addData("trebuchet start", null);
                break;
            case TREBUCHET_UP1:
                if(trebuchet.moveToTarget(-175)) {
                    turretRotation = TurretRotation.TURRET_TORIGHT;
                }
                break;
            case TREBUCHET_TOHUB:
                if(trebuchet.moveToTarget(armTarget)) {
                    trebuchetPosition = TrebuchetPosition.TREBUCHET_HUB;
                }
                telemetry.addData("trebuchet to hub", null);
                break;
            case TREBUCHET_HUB:
                trebuchet.moveToTarget(armTarget);
                telemetry.addData("trebuchet hub", null);
                break;
            case TREBUCHET_UP2:
                if (trebuchet.moveToTarget(-175)) {
                    turretRotation = TurretRotation.TURRET_TOFRONT;
                }
                break;
            case TREBUCHET_TOLOWER:
                if(trebuchet.moveToTarget(0)) {
                    trebuchetPosition = TrebuchetPosition.TREBUCHET_LOWER;
                }
                telemetry.addData("trebuchet to lower", null);
                break;
            case TREBUCHET_LOWER:
                complete[3] = true;
                telemetry.addData("trebuchet lower", null);
                break;
            default:
                trebuchetPosition = TrebuchetPosition.TREBUCHET_START;
        }
        switch (scoopState) {
            case SCOOP_START:
                telemetry.addData("scoop start", null);
                break;
            case SCOOP_ON:
                //Intake already flipped from earlier
                toggles.flipIntake();
                if (timer.milliseconds() >= scoopTime) {
                    drivePosition = DrivePosition.POSITION_TODEPOT;
                    scoopState = ScoopState.SCOOP_OFF;
                    markerS = true;
                }
                telemetry.addData("scoop on", null);
                break;
            case SCOOP_OFF:
                toggles.stopIntake();
                complete[4] = true;
                telemetry.addData("scoop off", null);
                break;
            default:
                scoopState = ScoopState.SCOOP_START;
        }
        telemetry.update();
        if (complete.equals(new boolean[] {true, true, true, true, true})) {
            stop();
        }
    }
}
