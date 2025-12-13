package org.firstinspires.ftc.teamcode; // make sure this aligns with class location



import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.jumpypants.murphy.states.StateMachine;
import com.jumpypants.murphy.tasks.Task;
import org.firstinspires.ftc.teamcode.MyRobot;
import com.jumpypants.murphy.util.RobotContext;


import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;




@Autonomous(name = "Example Auto", group = "Examples")
public class MainAuton extends OpMode {

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    private Task initializeIntakeTask;

    private MyRobot robotContext;


    public enum PathState {
        //Start Position - End Position
        //Drive > MoveMoment State
        //Shoot > attempt to score the artifact

        STARTPOSE,
        STARTPOSE_PICKUPOSE1,
        PICKUPPOSE1,
        PICKUPPOSE1_LAUNCHPOSE1,
        LAUNCHPOSE1
    }


    PathState pathState;

    private final Pose startPose = new Pose(66.66666666666667, 9.422222222222222);

    private final Pose pickUpPose1 = new Pose(19.02222222222222, 36.088888888888896);

    private final Pose launchPose1 = new Pose(72.000, 22.400);



    private PathChain startPosePickUpPose1, pickUpPose1LaunchPose1;




    public void buildPaths() {
        startPosePickUpPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pickUpPose1))
                .setTangentHeadingInterpolation()
                .build();

        pickUpPose1LaunchPose1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickUpPose1,launchPose1))
                .setTangentHeadingInterpolation()
                .build();

    }

    public void statePathUpdate() {
        switch(pathState) {
            case STARTPOSE:
                if (!follower.isBusy()) {
                    //add Intake start logic
                    robotContext.INTAKE.setAutoPower(1);
                    telemetry.addLine("Started intake");
                    setPathState(PathState.STARTPOSE_PICKUPOSE1);
                }
                break;
            case STARTPOSE_PICKUPOSE1:
                follower.followPath(startPosePickUpPose1, true);
                setPathState(PathState.PICKUPPOSE1); //Reset the timer and make new state
                break;
            case PICKUPPOSE1:
                if (!follower.isBusy()) {
                    //add intake stop logic
                    robotContext.INTAKE.setAutoPower(0);
                    telemetry.addLine("stopped intake");
                    setPathState(PathState.PICKUPPOSE1_LAUNCHPOSE1);
                }
                break;
            case PICKUPPOSE1_LAUNCHPOSE1:
                follower.followPath(pickUpPose1LaunchPose1, true);
                setPathState(PathState.LAUNCHPOSE1); //Reset the timer and make new state
                break;
            case LAUNCHPOSE1:
                if (!follower.isBusy()) {
                    //Needs to be fixed

                    //Add Outtake start logic
                    robotContext.Shooter.autoRunShooter(1);

                    //Add run transfer logic
                    robotContext.TRANSFER.autoMoveLeftTask(0.5,2);
                    robotContext.TRANSFER.autoMoveRightTask(0.49,2);

                    robotContext.Shooter.autoRunShooter(0);


                    telemetry.addLine("Done with launch");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;

        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }







    @Override
    public void init() {
        //Add command to start running intake

        robotContext = new MyRobot(
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
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)

        );







        pathState = PathState.STARTPOSE_PICKUPOSE1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);


    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }
}
