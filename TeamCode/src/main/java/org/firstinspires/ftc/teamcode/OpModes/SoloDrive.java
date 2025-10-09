package org.firstinspires.ftc.teamcode.OpModes;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@TeleOp (name = "Solo Drive", group = "TeleOp")
public class SoloDrive extends OpMode {
    public static Pose startPose = new Pose();
    private Supplier<PathChain> pathChain;
    private static DcMotorEx turret = null;
    private static DcMotorEx flywheel1 = null;
    private static DcMotorEx flywheel2 = null;
    private static DcMotorEx intake = null;
    private static Servo kicker = null;
    private static Limelight3A limelight = null;
    private static final int TURRET_MIN = -325;
    private static final int TURRET_MAX = 325;
    private static final double TICKS_PER_DEGREE = 2.11604166667*(1150/435); // tune this
    public static double strength = 1.2;
    public double lastTx = 0;
    public static double p = 650;
    public static double i = 0;
    public static double d = 10;
    public static double f = 0;
    public static int vel = 1350;
    public static int pos = 0;
    public static double pow = 0;
    public static int refreshRate = 200;
    public static double distanceConst = 20.625;
    public static double power = 17;
    public static int targetID = 0;
    private static int id;
    public static Follower follower;
    public void init() {
        // Initialize the Drivetrain motors. Make sure the deviceName matches the name of the motor on the Driver Hub.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");

        //Reverse other motors
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set up the PIDs
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(10);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(refreshRate);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98,0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    public void init_loop() {

    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        limelight.setPollRateHz(refreshRate);
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            id = fr.getFiducialId();
        }
        if (result.isValid() && id == targetID) {
            double tx = result.getTx();
            double ty = result.getTy();// horizontal error in degrees
            lastTx = 0;

            if(Math.abs(result.getTx()) > 23) {
                lastTx = result.getTx();
            }

            // Convert error into encoder ticks
            int adjustment = (int)(tx * TICKS_PER_DEGREE);

            // Update turret target
            pos += (int)(strength*adjustment/10);

            // Clamp to turret limits
            pos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, pos));

            // Apply
            turret.setTargetPosition(pos);

            telemetry.addData("Target Detected", true);
            telemetry.addData("tx", tx);
            telemetry.addData("Turret Target", pos);
        } else {
            telemetry.addData("Target Detected", false);
            pos = 0;
        }

        flywheel2.setPower(flywheel1.getPower());
        flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
        flywheel1.setVelocity(vel);

        turret.setPositionPIDFCoefficients(power);
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        flywheel1.setPower(pow);
        flywheel2.setPower(flywheel1.getPower());

        if(gamepad1.x) {
            intake.setPower(1);
        } else {
            intake.setPower(-1);
        }

        if(gamepad1.a) {
            kicker.setPosition(0);
            intake.setPower(0);
        } else {
            kicker.setPosition(0.23);
        }

        if(gamepad1.dpad_left) {
            targetID = 20;
        } else if(gamepad1.dpad_right) {
            targetID = 24;
        }

        if(gamepad2.dpad_down) {
            vel = 1350;
        } else if(gamepad2.dpad_up) {
            vel = 1600;
        }
    }
}
