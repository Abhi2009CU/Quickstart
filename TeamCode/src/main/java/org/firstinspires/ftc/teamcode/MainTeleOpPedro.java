package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="TELEOP_WITH_PEDRO", group="Linear OpMode")
public class MainTeleOpPedro extends LinearOpMode {

    StateMachine stateMachine;
    Follower follower = Constants.createFollower(hardwareMap);
    Turret turret;
    private TelemetryManager telemetryM;

    public static double targetX = 144;
    public static double targetY = 144;
    public static Pose startingPose = new Pose(72, 72, 0.5 * Math.PI);
    public static int alliance;

    @Override
    public void runOpMode() {
        while (opModeInInit()){
            if (gamepad1.a){
                alliance = 0;
                telemetry.addLine("Alliance: Blue");
            } else if (gamepad1.b) {
                alliance = 1;
                telemetry.addLine("Alliance: Red");
            }
            telemetry.update();

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            follower.setStartingPose(startingPose);
            follower.update();

            MyRobot robotContext = new MyRobot(
                    telemetry,
                    gamepad1,
                    gamepad2,
                    follower,
                    new Intake(hardwareMap),
                    new Shooter(hardwareMap),
                    new Transfer(hardwareMap),
                    new Turret(hardwareMap)
            );
            stateMachine = new StateMachine(new IntakingState(robotContext), robotContext);

            follower.startTeleopDrive();
        }

        targetX *= alliance;

        while (opModeIsActive()){
            stateMachine.step();

            follower.update();

            Pose currentPose = follower.getPose();

            turret.getCurrentRotation();
            turret.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            turret.updatePID();


            telemetry.addData("Current Rotation (rad)", turret.getCurrentRotation());
            telemetry.addData("Target Rotation (rad)", Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            telemetry.update();
        }
    }
}