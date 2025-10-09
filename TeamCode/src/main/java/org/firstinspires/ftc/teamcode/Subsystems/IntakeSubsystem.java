package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    public enum IntakeState {
        INTAKE, STOP, REVERSE, PARTIALINTAKE
    }
    public IntakeState istate;
    public Timer kickerTimer;
    private DcMotorEx intake;
    private Servo kicker;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");

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

    public void partialIntake() {
        setIntakeState(IntakeState.PARTIALINTAKE);
    }

    public void kick() {
        for(int i = 0; i<= 2; i++) {
            if(kickerTimer.getElapsedTimeSeconds() > 0.5) {
                kicker.setPosition(0.02);
                if(kickerTimer.getElapsedTimeSeconds() > 0.675) {
                    kicker.setPosition(0.23);
                    kickerTimer.resetTimer();
                }
            }
        }
    }

    public void set() {
        kicker.setPosition(0.23);
    }

    public void update() {
        intakeState();
    }
}
