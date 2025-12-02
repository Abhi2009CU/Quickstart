package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MyRobot;
public class Intake {
    private final Motor IntakeMotor;
    public final int MAXPOWER = 1, STOPPOWER = 0;




    public Intake(HardwareMap hardwareMap) {
        IntakeMotor = new Motor(hardwareMap, "IntakeMotor");
        IntakeMotor.setRunMode(Motor.RunMode.RawPower);
        IntakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    public class SetIntakePower extends Task {
        private final double POWER;

        public SetIntakePower(MyRobot robotContext, double power) {
            super(robotContext);
            this.POWER = power;
        }


        @Override
        protected void initialize(RobotContext robotContext) {
            IntakeMotor.set(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) { //to be cont.
            return false;
        }
    }



    public class ManualRunIntakeMotor extends Task {
        private boolean buttonClick = false;
        private boolean lastButtonState = false;

        public ManualRunIntakeMotor (RobotContext robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {

        }

        @Override
        protected boolean run(RobotContext robotContextWrapper) {
            MyRobot robotContext = (MyRobot) robotContextWrapper;
            if (robotContext.gamepad1.x) {
                IntakeMotor.set(MAXPOWER);
            } else {
                IntakeMotor.set(STOPPOWER);
            }

            if (robotContext.gamepad1.y) { //For when we want to stop intaking and start transfer
                return false;
            }

            return true;
        }
    }
}