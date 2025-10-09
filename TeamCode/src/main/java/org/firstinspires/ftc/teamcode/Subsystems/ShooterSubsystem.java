package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class ShooterSubsystem {
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private Limelight3A limelight = null;
    private static final int TURRET_MIN = -325;
    private static final int TURRET_MAX = 325;
    private static final double TICKS_PER_DEGREE = 2.11604166667*(1150/435); // tune this
    public static double strength = 1.2;
    public static int pos = 0;
    public double lastTx = 0;
    public static double p = 650;
    public static double i = 0;
    public static double d = 10;
    public static double f = 0;
    public static int targetID = 0;
    private static int id;

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

    public void telemetry() {
    }
}
