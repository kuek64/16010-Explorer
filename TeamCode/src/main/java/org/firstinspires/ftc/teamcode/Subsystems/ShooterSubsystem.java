package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class ShooterSubsystem {
    public static double FLYWHEEL_VELOCITY_SLOPE = 9;     // velocity change per inch (adjustable)
    public static double FLYWHEEL_VELOCITY_OFFSET = 1200;   // base velocity
    public static double FLYWHEEL_MIN_VELOCITY = 1200;
    public static double FLYWHEEL_MAX_VELOCITY = 1600;
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 133.0;
    public static double redGoalX  = 144.0;
    public static double redGoalY  = 133.0;
    private int previousTurretPos = 0;
    private double previousFlywheelVel = 0;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    public static double strength = 1.2;
    public static double slope = -5.4;
    public static double offset = -140;
    public static double distanceScalar = -25;
    public static int pos = 0;
    public static double p = 650;
    public static double i = 0;
    public static double d = 10;
    public static double f = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setTurretPosition(int pos) {
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
    }

    // You may want to store previousPos as a class variable
    private int previousPos = 0;

    public void alignTurret(double x, double y, double headingDeg, boolean blue, Telemetry telemetry) {
        // ---- Field goal coordinates ----
        headingDeg = Math.toDegrees(headingDeg);

        // ---- Turret physical offset (4 inches behind robot along Y-axis) ----
        double turretOffsetY = 4.0;

        // ---- Pick correct goal ----
        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double dx = goalX - (x+x/distanceScalar);
        double dy = goalY - ((y - turretOffsetY +y/distanceScalar));

        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        double turretAngle;
        if (blue) {
            turretAngle = angleToGoal - headingDeg;
        } else {
            turretAngle = (angleToGoal + 180) - headingDeg;
        }

        turretAngle = (turretAngle + 540) % 360 - 180;

        int targetTicks = (int) (slope * turretAngle + offset);

        // ---- Step 6: Clamp to turret limits ±500 ticks ----
        final int TURRET_MIN = -650;
        final int TURRET_MAX = 650;
        targetTicks = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTicks));

        pos = previousPos + (int)(0.1 * (targetTicks - previousPos));
        previousPos = pos;

        // ---- Step 8: Compute distance to goal ----
        double distance = Math.abs(Math.hypot(dx, dy));

        double targetVelocity = FLYWHEEL_VELOCITY_OFFSET + (distance - 72) * FLYWHEEL_VELOCITY_SLOPE; // 72 as reference distance
        targetVelocity = Math.max(FLYWHEEL_MIN_VELOCITY, Math.min(FLYWHEEL_MAX_VELOCITY, targetVelocity));

        // ---- Step 10: Smooth flywheel velocity using global rate ----
        double smoothedVelocity = previousFlywheelVel + (targetVelocity - previousFlywheelVel);
        previousFlywheelVel = smoothedVelocity;
        setFlywheelVelocity((int) smoothedVelocity);

        // ---- Send target to turret ----
        setFlywheelVelocity((int) smoothedVelocity);
        setTurretPosition(pos);

        // ---- Telemetry debug info ----
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("angleToGoal", angleToGoal);
        telemetry.addData("turretAngle (normalized)", turretAngle);
        telemetry.addData("headingDeg", headingDeg);
        telemetry.addData("targetTicks (raw)", targetTicks);
        telemetry.addData("turretCurrent", turret.getCurrentPosition());
        telemetry.addData("pos (smoothed)", pos);
        telemetry.update();
    }



    public int getPos() {
        return turret.getCurrentPosition();
    }
    public void telemetry() {
    }
}
