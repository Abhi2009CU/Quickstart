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

        mainTask = new SequentialTask(robotContext,
                new ParallelTask(robotContext, true,
                        robotContext.TRANSFER.new ManualControlTask(robotContext),
                        robotContext.INTAKE.new ManualRunIntakeMotor(robotContext)
                )
        );
    }

    @Override
    public State step() {

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
