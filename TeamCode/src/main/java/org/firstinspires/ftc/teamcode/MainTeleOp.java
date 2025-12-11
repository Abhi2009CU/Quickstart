package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public enum StartingPositionMode {
        CLOSE,
        FAR,
        CARRY_OVER
    }

    StateMachine stateMachine;

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_TARGET_X = 14;
    public static double BLUE_TARGET_Y = 135;

    public static double BLUE_CLOSE_STARTING_X = 72;
    public static double BLUE_CLOSE_STARTING_Y = 72;
    public static double BLUE_CLOSE_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_FAR_STARTING_X = 72;
    public static double BLUE_FAR_STARTING_Y = 72;
    public static double BLUE_FAR_STARTING_HEADING = 0.5 * Math.PI;

    public static Alliance alliance = Alliance.BLUE;
    public static StartingPositionMode startingPositionMode = StartingPositionMode.CARRY_OVER;

    @Override
    public void runOpMode() {
        MyRobot robotContext = new MyRobot(
                telemetry,
                gamepad1,
                gamepad2,
                new Intake(hardwareMap),
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)
        );

        Follower follower = MyRobot.follower;

        stateMachine = new StateMachine(new IntakingState(robotContext), robotContext);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            if (gamepad2.left_trigger > 0.2){
                startingPositionMode = StartingPositionMode.CLOSE;
            } else if (gamepad2.right_trigger > 0.2) {
                startingPositionMode = StartingPositionMode.FAR;
            } else if (gamepad2.cross) {
                startingPositionMode = StartingPositionMode.CARRY_OVER;
            }

            telemetry.addLine("Starting Position: " + startingPositionMode.name());

            telemetry.update();
        }


        // If the alliance is BLUE, use the positions as
        // If it is RED, mirror the x coordinate about x = 72 (the field centerline)
        // Also if it is RED, invert the heading
        // Carry over means we don't need to set the starting pose here

        Pose startingPose;
        switch (startingPositionMode) {
            case CLOSE:
                startingPose = new Pose(
                        (alliance == Alliance.BLUE) ? BLUE_CLOSE_STARTING_X : 144 - BLUE_CLOSE_STARTING_X,
                        BLUE_CLOSE_STARTING_Y,
                        (alliance == Alliance.BLUE) ? BLUE_CLOSE_STARTING_HEADING : -BLUE_CLOSE_STARTING_HEADING
                );
                follower.setStartingPose(startingPose);
                break;
            case FAR:
                startingPose = new Pose(
                        (alliance == Alliance.BLUE) ? BLUE_FAR_STARTING_X : 144 - BLUE_FAR_STARTING_X,
                        BLUE_FAR_STARTING_Y,
                        (alliance == Alliance.BLUE) ? BLUE_FAR_STARTING_HEADING : -BLUE_FAR_STARTING_HEADING
                );
                follower.setStartingPose(startingPose);
                break;
        }

        double targetX = (alliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = (alliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;

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