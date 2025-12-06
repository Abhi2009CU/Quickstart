package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;

@Configurable
@TeleOp(name="Flap Pose Tuner", group="Debug")
public class FlapPoseTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 0.0;

    public static double LEFT_UP = 0.5;
    public static double LEFT_DOWN = 0.15;

    public static double RIGHT_UP = 0.49;
    public static double RIGHT_DOWN = 0.83;

    private TelemetryManager telemetryM;

    private Servo left;
    private Servo right;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        left = hardwareMap.get(Servo.class, "LeftFlap");
        right = hardwareMap.get(Servo.class, "RightFlap");


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVel(TARGET_VELOCITY);
            shooter.updatePID();

            if (gamepad1.a) {
                left.setPosition(LEFT_UP);
                right.setPosition(RIGHT_UP);
            } else {
                left.setPosition(LEFT_DOWN);
                right.setPosition(RIGHT_DOWN);
            }

            telemetryM.debug("Current Velocity (ticks/s)", shooter.getVelocity());
            telemetryM.debug("Target Velocity (ticks/s)", TARGET_VELOCITY);

            telemetryM.update();
        }
    }
}