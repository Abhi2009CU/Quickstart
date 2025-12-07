package org.firstinspires.ftc.teamcode.robotStates;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;

public class ShootingState implements State {

    private final MyRobot robotContext;
    private final Task mainTask;

    public ShootingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(
                robotContext,
                new ParallelTask(robotContext, true,
                        robotContext.Shooter.new SetHoodPosTask(robotContext, 0.75), //example hood position
                        robotContext.Shooter.new setVelocity(robotContext, Shooter.maxVelocity)
                ),
                robotContext.TRANSFER.new TransferTask(robotContext),
                robotContext.Shooter.new setVelocity(robotContext, Shooter.stopVelocity)
        );
    }

    @Override
    public State step() {
        Gamepad gamepad1 = robotContext.gamepad1;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Field Centric
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
