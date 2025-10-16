package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp (name = "Solo Drive", group = "TeleOp")
public class SoloDrive extends OpMode {
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    public static Follower follower;
    public static Pose resetPose = new Pose(72,72,90);
    public static Pose autoEndPose;
    private PathChain pathChain;
    public static int vel = 1350;

    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose == null ? new Pose() : autoEndPose);
        follower.update();

        pathChain = follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98,0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        follower.update();
        shooter.update();
        intake.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        shooter.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), true, telemetry);

        if(gamepad1.x) {
            intake.reverse();
        } else {
            intake.intake();
        }

        if(gamepad1.a) {
            intake.kick();
            intake.stop();
        } else {
            intake.set();
        }

        if(gamepad1.right_stick_button) {
            follower.setPose(resetPose);
        }

        if(gamepad1.dpad_down) {
            vel = 1350;
        } else if(gamepad1.dpad_up) {
            vel = 1600;
        }

        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Turret Pos: ", shooter.getPos());
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}