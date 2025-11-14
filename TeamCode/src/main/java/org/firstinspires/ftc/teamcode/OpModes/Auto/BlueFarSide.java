package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.BlueTwelveArtifact.autoEndPose;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Blue Far", group = "Autonomous")
public class BlueFarSide extends OpMode {
    public static Follower follower;

    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(62, 10, Math.toRadians(90));
    private final Pose scorePose = new Pose(62, 10, Math.toRadians(90));
    private final Pose pickup1Pose = new Pose(10.75, 13, Math.toRadians(225));
    private final Pose intake1Pose = new Pose(10.75, 19, Math.toRadians(225));
    private final Pose parkPose = new Pose(36, 14, Math.toRadians(90));

    private PathChain scorePickup1, grabPickup1 , leave;

    public void buildPaths() {
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intake1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .addPath(new BezierLine(intake1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStrength(0.5)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.stop();
                shooter.setFlywheelVelocity(1500);
                shooter.setTurretPosition(-125);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.75) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 4.55) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2.2) {
                    intake.stop();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 2.4) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 2.2) {
                    intake.stop();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 2.4) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                    intake.partialintake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 2.2) {
                    intake.stop();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 2.4) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.5) {
                    shooter.setTurretPosition(0);
                    shooter.setFlywheelVelocity(0);
                    follower.followPath(leave);
                    setPathState(-1);
                }
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
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void loop() {
        intake.overIntake();
        follower.update();
        shooter.update();
        intake.update();
        autonomousPathUpdate();
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}
