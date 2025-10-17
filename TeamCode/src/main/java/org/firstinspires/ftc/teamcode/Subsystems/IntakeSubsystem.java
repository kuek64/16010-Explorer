package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {
    public enum IntakeState {
        INTAKE, STOP, REVERSE, PARTIALINTAKE
    }
    public IntakeState istate;
    public int kState;
    public Timer kickerTimer, kTimer;
    private DcMotorEx intake;
    private Servo kicker;
    private DistanceSensor dsensor;
    private boolean kBoolean = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        dsensor = hardwareMap.get(DistanceSensor.class, "csensor");

        kickerTimer = new Timer();
        kTimer = new Timer();
    }

    public void intakeState() {
        if(istate == IntakeState.INTAKE) {
            intake.setPower(1);
            istate = IntakeState.INTAKE;
        } else if(istate == IntakeState.STOP) {
            intake.setPower(0);
            istate = IntakeState.STOP;
        } else if(istate == IntakeState.REVERSE) {
            intake.setPower(-1);
            istate = IntakeState.REVERSE;
        } else if(istate == IntakeState.PARTIALINTAKE) {
            intake.setPower(0.85);
        }
    }

    public void setIntakeState(IntakeState state) {
        istate = state;
    }

    public void intake() {
        setIntakeState(IntakeState.INTAKE);
    }

    public void stop() {
        setIntakeState(IntakeState.STOP);
    }

    public void reverse() {
        setIntakeState(IntakeState.REVERSE);
    }

    public void partialintake() {
        setIntakeState(IntakeState.PARTIALINTAKE);
    }

    public void kickSequence() {
        double distance = dsensor.getDistance(DistanceUnit.INCH);

        if (distance <= 3 && kState == -1) {
            kickerSeriesStart();
        }
    }

    public void kickerSeriesStart() {
        setKickState(0);
    }

    public void setKickState(int state) {
        kState = state;
        kTimer.resetTimer();
    }

    public void kickSeries() {
        switch (kState) {
            case 0:
                stop();
                setKickState(1);
                break;

            case 1:
                if (kTimer.getElapsedTimeSeconds() > 0.05) {
                    kick();
                    setKickState(2);
                }
                break;

            case 2:
                if (kTimer.getElapsedTimeSeconds() > 0.325) {
                    set();
                    setKickState(3);
                }
                break;

            case 3:
                if (kTimer.getElapsedTimeSeconds() > 0.2) {
                    intake();
                    setKickState(-1);
                }
                break;

            case -1:
            default:
                intake();
                break;
        }
    }

    public double getDistance() {
        return dsensor.getDistance(DistanceUnit.INCH);
    }

    public void kick() {
        kicker.setPosition(0);
    }

    public void set() {
        kicker.setPosition(0.23);
    }

    public void update() {
        intakeState();
        kickSeries();
    }
}
