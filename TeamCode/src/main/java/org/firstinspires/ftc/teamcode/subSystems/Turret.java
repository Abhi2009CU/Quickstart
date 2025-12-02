package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private final Motor turretMotor;
    private final PIDController pid;

    private static final double P_GAIN = 0.001;
    private static final double I_GAIN = 0.0001;
    private static final double D_GAIN = 0.004;

    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1.0;

    private static final double TICKS_PER_REV = 288.0;

    private double targetPosition = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = new Motor(hardwareMap, "turretMotor");
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.resetEncoder();
        pid = new PIDController(P_GAIN, I_GAIN, D_GAIN);
    }

    /**
     * <p>Calculate the angle needed for the robot's turret to aim to a specific position on the field.<br />Note that <span style="font-style:italic">either degrees or radians can be used</span>, but you have to be consistent.<br /><br />&quot;I &lt;3 trigonometry!&quot;</p>
     * @param cX The current X position of Mr. Roboto
     * @param cY The current Y position of Mr. Roboto
     * @param cR The current rotation of Mr. Roboto
     * @param tX The target's X position
     * @param tY The target's Y position
     * @return The target angle for Mr. Roboto's turret
     */
    public static double calculateTurretAngle(double cX, double cY, double cR, double tX, double tY) {
        double a = tX - cX;
        double b = tY - cY;
        double c = Math.sqrt((a * a) + (b * b));
        double A = Math.asin(1 / c * a);
        // double B = Math.asin(1 / c * b);
        double target = cR - A;
        return target;
    }

    public void setPosition(double newTarget) {
        double currentPosition = getCurrentPosition();
        double error = newTarget - currentPosition;

        // error is how the turret finds the shortest path and how PID works
        error = (error + TICKS_PER_REV / 2) % TICKS_PER_REV - TICKS_PER_REV / 2;

        double proposedTarget = currentPosition + error;

        double maxLimit = TICKS_PER_REV;
        double minLimit = -TICKS_PER_REV;

        if (proposedTarget > maxLimit) {
            proposedTarget -= TICKS_PER_REV; // take long way around
        } else if (proposedTarget < minLimit) {
            proposedTarget += TICKS_PER_REV; // take long way around
        }

        targetPosition = proposedTarget;
        pid.setSetPoint(targetPosition);
    }

    public void updatePID() {
        double currentPosition = getCurrentPosition();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, MAX_POWER), MIN_POWER);
        turretMotor.set(power);
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.set(0);
    }

    public class RotateTurretTask extends Task {
        private static final double TOLERANCE = 5.0;
        // TOLERANCE is the small acceptable error range in encoder ticks

        private final double TARGET_POSITION;

        public RotateTurretTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            this.TARGET_POSITION = targetPosition;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            Turret.this.setPosition(TARGET_POSITION);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            updatePID();
            return Math.abs(Turret.this.getCurrentPosition() - TARGET_POSITION) > TOLERANCE;
        }
    }
}
