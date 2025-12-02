package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.states.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Launcher;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;


@TeleOp(name="InfernoDecode Tele-Op", group="Linear OpMode")
public class MainTeleOp extends OpMode {
    private StateMachine stateMachine;

    @Override
    public void init() {
        MyRobot robotContext = new MyRobot(
                telemetry,
                gamepad1,
                gamepad2,
                new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                        hardwareMap.get(Motor.class, "frontLeft"),
                        hardwareMap.get(Motor.class, "backLeft"),
                        hardwareMap.get(Motor.class, "frontRight"),
                        hardwareMap.get(Motor.class, "backRight")
                ),

                new Intake(hardwareMap),
                new Launcher(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)

        );



        stateMachine = new StateMachine(new IntakingState(robotContext), robotContext);


    }

    @Override
    public void loop() {
        stateMachine.step();
    }
}