package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Configurable
@TeleOp(name = "Turret Tuning", group = "TeleOp")
public class TurretTest extends OpMode {
    ShooterSubsystem shooter;
    public static int pos = 0;
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
    }

    public void loop() {
        shooter.update();

        shooter.setTurretPosition(pos);

        telemetry.addData("Velocity: ", shooter.getPos());
        telemetry.addData("Velocity Target: ", pos);
    }
}
