package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Blue 9+3", group = "Autonomous")
public class TwelveArtifacts extends OpMode {
    public static Follower follower;
    public static Pose autoEndPose;

    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(28, 127, Math.toRadians(180));
    private final Pose scorePose = new Pose(60, 83.5, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(21, 84, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(42, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(14, 59, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(42, 59, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(14, 36, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(42, 36, Math.toRadians(180));
    private final Pose pickup1PoseC = new Pose(85, 72, Math.toRadians(180));
    private final Pose pickup2PoseC = new Pose(100, 59, Math.toRadians(180));
    private final Pose pickup3PoseC = new Pose(100, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain scorePickup1, scorePickup2, scorePickup3, grabPickup1, grabPickup2, grabPickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .addPath(new BezierCurve(intake1Pose, pickup1PoseC, pickup1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .addPath(new BezierCurve(intake2Pose, pickup2PoseC, pickup2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading())
                .addPath(new BezierCurve(intake3Pose, pickup3PoseC, pickup3Pose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.setFlywheelVelocity(1250);
                shooter.setTurretPosition(0);
                intake.stop();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 5.5) {
                    intake.intake();
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 5.5) {
                    intake.intake();
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 5.5) {
                    intake.intake();
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 2.75) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 5.5) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void loop() {
        follower.update();
        shooter.update();
        intake.update();
        autonomousPathUpdate();
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Intake State: ", intake.istate);
        telemetry.addData("Distance: ", intake.getDistance());
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}
