package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Autonomous with full path sequence and shooting
 * Hardware mapping:
 * - leftPelvis/rightPelvis → shooterMotor "Sh1"
 * - inter → spindexerMotor "spin"
 * - pickUp → feederServo "feeder" (cycling)
 */
@Autonomous(name = "autoRed1", group = "auto-pedro")
public class autoRed1 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, feederTimer;

    private int pathState;

    private Paths paths;
    private final Pose startPose = new Pose(84, 12, Math.toRadians(90));

    // ===== HARDWARE =====
    private DcMotorEx shooterMotor;    // "Sh1" - was leftPelvis/rightPelvis
    private DcMotorEx spindexerMotor;  // "spin" - was inter
    private Servo feederServo;         // "feeder" - was pickUp

    // ===== CONSTANTS =====
    private static final int SHOOTER_VELOCITY = 1650;     // Same as leftPelvis/rightPelvis
    private static final int SPINDEXER_VELOCITY = 1900;   // Same as inter
    private static final int SPINDEXER_SLOW = 300;        // Slow feed velocity

    private static final double FEEDER_DOWN = 0.0;
    private static final double FEEDER_UP = 0.667;
    private static final double FEEDER_CYCLE_TIME = 0.3;

    private boolean feederUp = false;

    public static class Paths {
        public PathChain launch1;
        public PathChain prePickUp1;
        public PathChain pickUp1;
        public PathChain launch2;
        public PathChain prePickUp2;
        public PathChain pickUp2;
        public PathChain launch3;
        public PathChain prePickUp3;
        public PathChain pickUp3;
        public PathChain launch4;

        public Paths(Follower follower) {
            launch1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(84.000, 12.000),
                            new Pose(100.000, 100.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45)).build();

            prePickUp1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(100.000, 100.000),
                            new Pose(95.000, 83.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            pickUp1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(95.000, 83.500),
                            new Pose(125.000, 83.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            launch2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(125.000, 83.500),
                            new Pose(100.000, 100.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            prePickUp2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(100.000, 100.000),
                            new Pose(95.000, 60.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            pickUp2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(95.000, 60.000),
                            new Pose(130.000, 59.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            launch3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(130.000, 59.500),
                            new Pose(100.000, 100.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();

            prePickUp3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(100.000, 100.000),
                            new Pose(95.000, 35.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180)).build();

            pickUp3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(95.000, 35.500),
                            new Pose(130.000, 35.500)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            launch4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(130.000, 35.500),
                            new Pose(100.000, 100.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45)).build();
        }
    }

    // Helper method to cycle feeder servo during shooting
    private void cycleFeeder() {
        if (feederTimer.getElapsedTimeSeconds() > FEEDER_CYCLE_TIME / 2) {
            if (feederUp) {
                feederServo.setPosition(FEEDER_DOWN);
                feederUp = false;
            } else {
                feederServo.setPosition(FEEDER_UP);
                feederUp = true;
            }
            feederTimer.resetTimer();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.launch1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Turn on shooter (was leftPelvis/rightPelvis)
                    shooterMotor.setVelocity(SHOOTER_VELOCITY);
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 1 second for spinup
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    // Turn on spindexer (was inter) + start feeder cycling
                    spindexerMotor.setVelocity(SPINDEXER_VELOCITY);
                    feederServo.setPosition(FEEDER_UP);
                    feederUp = true;
                    feederTimer.resetTimer();
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Cycle feeder while shooting for 3 seconds
                cycleFeeder();
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    shooterMotor.setVelocity(0);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(4);
                }
                break;

            case 4:
                follower.followPath(paths.prePickUp1, true);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp1, true);
                    // Slow spindexer during pickup + feeder cycling
                    spindexerMotor.setVelocity(SPINDEXER_SLOW);
                    feederTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                cycleFeeder();
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch2, true);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    // Turn on shooter
                    shooterMotor.setVelocity(SHOOTER_VELOCITY);
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait 1 second for spinup
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    spindexerMotor.setVelocity(SPINDEXER_VELOCITY);
                    feederServo.setPosition(FEEDER_UP);
                    feederUp = true;
                    feederTimer.resetTimer();
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Shoot for 4 seconds
                cycleFeeder();
                if (actionTimer.getElapsedTimeSeconds() > 4) {
                    shooterMotor.setVelocity(0);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(10);
                }
                break;

            case 10:
                follower.followPath(paths.prePickUp2, true);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp2, true);
                    spindexerMotor.setVelocity(SPINDEXER_SLOW);
                    feederTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                cycleFeeder();
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch3, true);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    shooterMotor.setVelocity(SHOOTER_VELOCITY);
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    spindexerMotor.setVelocity(SPINDEXER_VELOCITY);
                    feederServo.setPosition(FEEDER_UP);
                    feederUp = true;
                    feederTimer.resetTimer();
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                cycleFeeder();
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    shooterMotor.setVelocity(0);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(16);
                }
                break;

            case 16:
                follower.followPath(paths.prePickUp3, true);
                setPathState(17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickUp3, true);
                    spindexerMotor.setVelocity(600);  // Faster pickup for last one
                    feederTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                cycleFeeder();
                if (!follower.isBusy()) {
                    follower.followPath(paths.launch4, true);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    shooterMotor.setVelocity(SHOOTER_VELOCITY);
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20:
                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    spindexerMotor.setVelocity(SPINDEXER_VELOCITY);
                    feederServo.setPosition(FEEDER_UP);
                    feederUp = true;
                    feederTimer.resetTimer();
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21:
                cycleFeeder();
                if (actionTimer.getElapsedTimeSeconds() > 3.0) {
                    shooterMotor.setVelocity(0);
                    spindexerMotor.setVelocity(0);
                    feederServo.setPosition(FEEDER_DOWN);
                    setPathState(-1);  // End autonomous
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shooter vel", shooterMotor.getVelocity());
        telemetry.addData("spindexer vel", spindexerMotor.getVelocity());
        telemetry.addData("feeder", feederUp ? "UP" : "DOWN");
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        feederTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);

        // Initialize shooter motor (was leftPelvis/rightPelvis)
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Sh1");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize spindexer motor (was inter)
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spin");
        spindexerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize feeder servo (was pickUp behavior)
        feederServo = hardwareMap.get(Servo.class, "feeder");
        feederServo.setDirection(Servo.Direction.REVERSE);
        feederServo.setPosition(FEEDER_DOWN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setVelocity(0);
        if (spindexerMotor != null) spindexerMotor.setVelocity(0);
        if (feederServo != null) feederServo.setPosition(FEEDER_DOWN);
    }
}
