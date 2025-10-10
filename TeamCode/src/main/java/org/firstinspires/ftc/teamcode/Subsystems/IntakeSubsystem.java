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
    public Timer kickerTimer;
    private DcMotorEx intake;
    private Servo kicker;
    private DistanceSensor dsensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        dsensor = hardwareMap.get(DistanceSensor.class, "csensor");

        kickerTimer = new Timer();
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
            intake.setPower(0.4);
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

    public void kickSequence() {
        for(int i = 0; i<= 2; i++) {
            if(dsensor.getDistance(DistanceUnit.INCH) < 1) {
                stop();
                kick();
                if(kickerTimer.getElapsedTimeSeconds() > 0.675) {
                    intake();
                    set();
                    kickerTimer.resetTimer();
                }
            } else {
                kickerTimer.resetTimer();
            }
        }
    }

    public void kick() {
        kicker.setPosition(0);
    }

    public void set() {
        kicker.setPosition(0.23);
    }

    public void update() {
        intakeState();
    }
}
