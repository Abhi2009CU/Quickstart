package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public IntakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask =  new SequentialTask(robotContext,
                robotContext.INTAKE.new ManualRunIntakeMotor(robotContext)/*,
                new TransferTask(robotContext)*/
        );
    }


    @Override
    public State step() {

        Gamepad gamepad1 = robotContext.gamepad1;
        robotContext.DRIVE.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (mainTask.step()) {
            return this;
        }

        return new ShootingState(robotContext, getName());
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}