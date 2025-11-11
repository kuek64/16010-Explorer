package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Red Close", group = "Autonomous")
public class RedTwelveArtifact extends OpMode {
    public static Follower follower;
    public static Pose autoEndPose;

    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose bluestartPose = new Pose(29.5, 127, Math.toRadians(180));
    private final Pose bluescorePose = new Pose(60, 83.5, Math.toRadians(180));
    private final Pose bluepickup1Pose = new Pose(27, 83.5, Math.toRadians(180));
    private final Pose blueintake1Pose = new Pose(52, 83.5, Math.toRadians(180));
    private final Pose bluepickup2Pose = new Pose(10, 60.5, Math.toRadians(180));
    private final Pose bluesetUpPose = new Pose(30, 77.5, Math.toRadians(90));
    private final Pose blueemptyPose = new Pose(16, 77.5, Math.toRadians(90));
    private final Pose blueintake2Pose = new Pose(60, 60.5, Math.toRadians(180));
    private final Pose bluepickup3Pose = new Pose(10, 34.5, Math.toRadians(180));
    private final Pose blueintake3Pose = new Pose(60, 34.5, Math.toRadians(180));
    private final Pose blueleavePose = new Pose(50, 73.5, Math.toRadians(135));

    private final Pose startPose = bluestartPose.mirror();
    private final Pose scorePose = bluescorePose.mirror();
    private final Pose pickup1Pose = bluepickup1Pose.mirror();
    private final Pose intake1Pose = blueintake1Pose.mirror();
    private final Pose pickup2Pose = bluepickup2Pose.mirror();
    private final Pose setUpPose = bluesetUpPose.mirror();
    private final Pose emptyPose = blueemptyPose.mirror();
    private final Pose intake2Pose = blueintake2Pose.mirror();
    private final Pose pickup3Pose = bluepickup3Pose.mirror();
    private final Pose intake3Pose = blueintake3Pose.mirror();
    private final Pose leavePose = blueleavePose.mirror();

    private Path scorePreload;
    private PathChain scorePickup1, scorePickup2, scorePickup3, grabPickup1, grabPickup2, grabPickup3, leave;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .addPath(new BezierLine(intake1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStrength(0.5)
                .addPath(new BezierLine(pickup1Pose, setUpPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), setUpPose.getHeading())
                .addPath(new BezierLine(setUpPose, emptyPose))
                .setLinearHeadingInterpolation(setUpPose.getHeading(), emptyPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .addPath(new BezierLine(intake2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading())
                .addPath(new BezierLine(intake3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.stop();
                shooter.setFlywheelVelocity(1220);
                shooter.setTurretPosition(-245);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 1.7) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.85) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1.6) {
                    intake.stop();
                }
                if(pathTimer.getElapsedTimeSeconds() > 3.3) {
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1.6) {
                    intake.kick();
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 3.9) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 2.55) {
                    intake.stop();
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 2.25) {
                    intake.kick();
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.85) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 3.5) {
                    intake.stop();
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 2.75) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 5.6) {
                    setPathState(8);
                }
                break;
            case 8:
                shooter.setTurretPosition(0);
                follower.followPath(leave);
                setPathState(-1);
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
