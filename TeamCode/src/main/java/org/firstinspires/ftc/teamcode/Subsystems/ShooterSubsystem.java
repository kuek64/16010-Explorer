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
    public static double fIntercept = 570;
    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 144;
    public static double redGoalY  = 144;
    public static int power = 17;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private RGBLight light = null;
    public static double tSlope = 5.56729166667;
    public static double fSlope = 3.878202325;
    public static int pos = 0;
    public static int vel = 0;
    public static double p = 200;
    public static double i = 0;
    public static double d = 60;
    public static double f = 20;
    public static double sp = 300;
    public static double si = 0;
    public static double sd = 50;
    public static double sf = 0;
    public static double cp = 200;
    public static double ci = 0;
    public static double cd = 10;
    public static double cf = 0;
    public static double counterFactor = -1.1;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        light = new RGBLight(hardwareMap.get(Servo.class, "light"));

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setVelocityPIDFCoefficients(cp, ci, cd, cf);
        flywheel2.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setTurretPosition(int pos) {
        turret.setPositionPIDFCoefficients(power);
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void setFlywheelVelocity(int vel) {
        /*if(Math.abs(flywheel2.getVelocity() - vel) > 100) {
            flywheel2.setVelocityPIDFCoefficients(p, i, d, f); }
        else {
            flywheel2.setVelocityPIDFCoefficients(sp, si, sd, sf);
        }*/
        flywheel2.setVelocityPIDFCoefficients(p,i,d,f);
        flywheel1.setVelocityPIDFCoefficients(cp,ci,cd,cf);
        flywheel2.setVelocity(vel);
        flywheel1.setVelocity(vel*counterFactor);
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

            final int TURRET_MIN = -1000;
            final int TURRET_MAX = 1000;

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
    public double getVel() {
        return flywheel2.getVelocity();
    }

    public double getVel2() {
        return flywheel1.getVelocity();
    }

    public void telemetry() {
    }
}
