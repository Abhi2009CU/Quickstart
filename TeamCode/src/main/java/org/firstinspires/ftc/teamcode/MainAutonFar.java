package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="TELEOP_WITH_PEDRO", group="Linear OpMode")
public class MainAutonFar extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_TARGET_X = 14;
    public static double BLUE_TARGET_Y = 135;

    public static double BLUE_STARTING_X = 72;
    public static double BLUE_STARTING_Y = 72;
    public static double BLUE_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_PRELOAD_X = 65;;
    public static double BLUE_PRELOAD_Y = 20;
    public static double BLUE_PRELOAD_HEADING = 0.5 * Math.PI;

    public static Alliance alliance = Alliance.BLUE;

    private Path goToPreload;

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

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            telemetry.update();
        }


        // If the alliance is BLUE, use the positions as
        // If it is RED, mirror the x coordinate about x = 72 (the field centerline)
        // Also if it is RED, invert the heading
        Pose startingPose;

        if (alliance == Alliance.BLUE) {
            startingPose = new Pose(
                    BLUE_STARTING_X,
                    BLUE_STARTING_Y,
                    BLUE_STARTING_HEADING
            );
        } else {
            startingPose = new Pose(
                    144 - BLUE_STARTING_X,
                    BLUE_STARTING_Y,
                    -BLUE_STARTING_HEADING
            );
        }

        follower.setStartingPose(startingPose);

        double targetX = (alliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = (alliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;

        follower.update();

        follower.startTeleopDrive(true);

        buildPaths(startingPose);

        Task mainTask = new SequentialTask(robotContext,
                getGoToPreloadTask(robotContext, follower),
                getShootThreeTask(robotContext)
        );

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2);
            robotContext.SHOOTER.setHoodByDistance(Math.sqrt(d));
            robotContext.SHOOTER.setVelByDistance(Math.sqrt(d));

            mainTask.step();
        }
    }

    private void buildPaths(Pose startingPose) {
        double preloadX, preloadY, preloadHeading;
        if (alliance == Alliance.BLUE) {
            preloadX = BLUE_PRELOAD_X;
            preloadY = BLUE_PRELOAD_Y;
            preloadHeading = BLUE_PRELOAD_HEADING;
        } else {
            preloadX = 144 - BLUE_PRELOAD_X;
            preloadY = BLUE_PRELOAD_Y;
            preloadHeading = -BLUE_PRELOAD_HEADING;
        }

        Pose preloadPose = new Pose(preloadX, preloadY, preloadHeading);

        goToPreload = new Path(new BezierLine(startingPose, preloadPose));
        goToPreload.setLinearHeadingInterpolation(startingPose.getHeading(), preloadPose.getHeading());
    }

    private Task getGoToPreloadTask(MyRobot robotContext, Follower follower) {
        return new Task(robotContext) {
            @Override
            protected void initialize(RobotContext robotContext) {
                follower.followPath(goToPreload);
            }

            @Override
            protected boolean run(RobotContext robotContext) {
                return follower.isBusy();
            }
        };
    }

    private Task getShootThreeTask(MyRobot robotContext){
        return new SequentialTask(robotContext,
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_UP_POS),
                robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_DOWN_POS),
                robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS),
                robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_UP_POS),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS)
                ),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_DOWN_POS),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS)
                ),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0)
        );
    }
}