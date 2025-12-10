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
public class MainTeleOp extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    StateMachine stateMachine;

    private Follower follower;
    private TelemetryManager telemetryM;

    private MyRobot robotContext;

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_TARGET_X = 14;
    public static double BLUE_TARGET_Y = 135;

    public static double RED_STARTING_X = 72;
    public static double RED_STARTING_Y = 72;
    public static double RED_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_STARTING_X = 72;
    public static double BLUE_STARTING_Y = 72;
    public static double BLUE_STARTING_HEADING = 0.5 * Math.PI;

    public static Alliance alliance;

    @Override
    public void runOpMode() {
        robotContext = new MyRobot(
                telemetry,
                gamepad1,
                gamepad2,
                follower,
                new Intake(hardwareMap),
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)
        );

        follower = Constants.createFollower(hardwareMap);

        stateMachine = new StateMachine(new IntakingState(robotContext), robotContext);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
                telemetry.addLine("Alliance: Blue");
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
                telemetry.addLine("Alliance: Red");
            }

            telemetry.update();
        }

        Pose startingPose = (alliance == Alliance.BLUE)
                ? new Pose(BLUE_STARTING_X, BLUE_STARTING_Y, BLUE_STARTING_HEADING)
                : new Pose(RED_STARTING_X, RED_STARTING_Y, RED_STARTING_HEADING);

        double targetX = (alliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = (alliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;

        follower.setStartingPose(startingPose);
        follower.update();

        follower.startTeleopDrive(true);

        while (opModeIsActive()){
            stateMachine.step();

            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2);
            robotContext.SHOOTER.setHoodByDistance(Math.sqrt(d));
            robotContext.SHOOTER.setVelByDistance(Math.sqrt(d));

            if (gamepad2.right_bumper) {
                robotContext.TURRET.incrementAngleOffset(0.01);
            }
            if (gamepad2.left_bumper) {
                robotContext.TURRET.incrementAngleOffset(-0.01);
            }

            if (gamepad2.right_trigger > 0.1) {
                robotContext.SHOOTER.incrementHoodOffset(0.01);
            }
            if (gamepad2.left_trigger > 0.1) {
                robotContext.SHOOTER.incrementHoodOffset(-0.01);
            }
            if (gamepad2.cross) {
                robotContext.SHOOTER.setHoodOffset(0);
            }

            follower.setTeleOpDrive(
                    Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(-gamepad1.left_stick_x, 3),
                    Math.pow(-gamepad1.right_stick_x, 3),
                    false // field Centric
            );
        }
    }
}