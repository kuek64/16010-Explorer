package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@Configurable
@TeleOp(name = "Flywheel Tuning", group = "TeleOp")
public class FlywheelTune extends OpMode {
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    public static int vel = 0;
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    public void loop() {
        shooter.update();

        shooter.setFlywheelVelocity(vel);

        telemetry.addData("Velocity: ", shooter.getVel());
        telemetry.addData("Velocity Counter: ", shooter.getVel2());
        telemetry.addData("Velocity Target: ", vel);
        telemetry.addData("Distance Sensor 1: ", intake.getDistance1());
        telemetry.addData("Distance Sensor 2: ", intake.getDistance2());
    }
}
