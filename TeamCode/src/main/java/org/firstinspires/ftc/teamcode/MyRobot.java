package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.jumpypants.murphy.util.RobotContext;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public final MecanumDrive DRIVE;
    public final Intake INTAKE;
    public final Shooter Shooter;
    public final Transfer TRANSFER;
    public final Turret TURRET;

    //not sure about the following
    public Gamepad gamepad1, gamepad2;

    /**
     * Creates a new RobotContext with the specified telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param telemetry the telemetry instance for driver station communication
     * @param gamepad1  the primary gamepad controller
     * @param gamepad2  the secondary gamepad controller
     */
    public MyRobot(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, MecanumDrive drive, Intake intake, Shooter shooter, Transfer transfer, Turret turret) {
        super(telemetry, gamepad1, gamepad2);
        this.DRIVE = drive;
        this.INTAKE = intake;
        this.Shooter = shooter;
        this.TRANSFER = transfer;
        this.TURRET = turret;

        //not sure about the following
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }
}