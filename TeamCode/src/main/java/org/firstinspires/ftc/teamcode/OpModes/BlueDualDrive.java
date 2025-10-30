package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.BlueTwelveArtifact.autoEndPose;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp (name = "Blue Dual Drive", group = "TeleOp")
public class BlueDualDrive extends OpMode {
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    public static Follower follower;
    public static Pose resetPose = new Pose(72,72,Math.toRadians(90));
    public static Pose parkPose = new Pose(110, 39.5, Math.toRadians(90));


    private Supplier<PathChain> pathChain;

    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose == null ? new Pose() : autoEndPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98,0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        intake.overIntake();
        follower.update();
        shooter.update();
        intake.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        shooter.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), true, telemetry);

        if(gamepad2.xWasPressed()) {
            intake.switchIntake();
        }

        if(gamepad2.a) {
            intake.kickSequenceTeleOp();
        }

        if(gamepad1.back) {
            follower.holdPoint(parkPose);
        }

        if(gamepad1.right_stick_button) {
            follower.setPose(resetPose);
        }

        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}