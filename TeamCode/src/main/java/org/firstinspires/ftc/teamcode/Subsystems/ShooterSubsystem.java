package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.atan;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class ShooterSubsystem {
    public static double turretOffsetY = -4.0;
    public static double turretOffsetX  = 0;
    public static double fIntercept = 874.35;
    public static double blueGoalX = 12;
    public static double blueGoalY = 130;
    public static double redGoalX  = 136;
    public static double redGoalY  = 130;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private RGBLight light = null;
    public static double tSlope = 5.60729166667;
    public static double fSlope = 4.568202325;
    public static int pos = 0;
    public static int vel = 0;
    public static double p = 1200;
    public static double i = 0;
    public static double d = 300;
    public static double f = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        light = new RGBLight(hardwareMap.get(Servo.class, "light"));

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setTurretPosition(int pos) {
        flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
        turret.setPositionPIDFCoefficients(17);
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void setFlywheelVelocity(int vel) {
        flywheel2.setPower(flywheel1.getPower());
        flywheel1.setVelocity(vel);
    }

    public void update() {
        ready();
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry) {
            double headingDeg = Math.toDegrees(heading);

            x = turretOffsetX + x;
            y = turretOffsetY + y;

            // ---- Pick correct goal ----
            double goalX = blue ? blueGoalX : redGoalX;
            double goalY = blue ? blueGoalY : redGoalY;

            double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));

            double turretAngle = angleToGoal + headingDeg - 90;

            int targetTicks = (int) (tSlope * turretAngle);

            final int TURRET_MIN = -600;
            final int TURRET_MAX = 600;

            if (targetTicks > TURRET_MAX || targetTicks < TURRET_MIN) {
                pos = 0;
            } else {
                pos = targetTicks;
            }

            double distance = Math.hypot(goalX - x, goalY - y);
            vel = (int) (distance * fSlope + fIntercept);

            setFlywheelVelocity(vel);
            setTurretPosition(pos);
    }

    public void ready() {
        if((Math.abs(turret.getCurrentPosition() - pos) < 5) && (Math.abs(flywheel1.getVelocity() - vel) < 80)) {
            light.green();
        } else {
            light.red();
        }
    }

    public int getPos() {
        return turret.getCurrentPosition();
    }
    public void telemetry() {
    }
}
