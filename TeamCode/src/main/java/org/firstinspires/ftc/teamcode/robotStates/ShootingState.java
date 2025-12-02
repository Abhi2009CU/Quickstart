package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;

public class ShootingState implements State {

    private final MyRobot robotContext;
    private final Task mainTask;

    public ShootingState(MyRobot robotContext, String previousStateName) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(
                robotContext,
                new ParallelTask(robotContext, true,
                        robotContext.LAUNCHER.new SetHoodPosTask(robotContext, 0.75), //example hood position
                        robotContext.LAUNCHER.new RunOuttakeTask(robotContext, 1.0)
                ),
                robotContext.TRANSFER.new TransferTask(robotContext),
                robotContext.LAUNCHER.new RunOuttakeTask(robotContext, 0.0)
        );
    }

    @Override
    public State step() {
        Gamepad gamepad1 = robotContext.gamepad1;
        robotContext.DRIVE.driveRobotCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        if (mainTask.step()) {
            return this;
        }

        return new IntakingState(robotContext);
    }


    @Override
    public String getName() {
        return "Shooting";
    }
}
