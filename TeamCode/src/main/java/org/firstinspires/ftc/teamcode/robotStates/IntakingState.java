package org.firstinspires.ftc.teamcode.robotStates;


import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public IntakingState(MyRobot robotContext) {
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

        return new ShootingState(robotContext);
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
