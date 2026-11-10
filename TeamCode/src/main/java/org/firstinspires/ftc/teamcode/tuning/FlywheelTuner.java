package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Flywheel tuning OpMode.
 *
 * OPEN-LOOP mode (default) - for finding kS and kV
 *   dpad up / down     : power +/- 0.01
 *   dpad right / left  : power +/- 0.10
 *   Telemetry shows the measured velocity and a kV estimate at the current power.
 *
 * CLOSED-LOOP mode (press A to toggle) - for checking feedforward and tuning kP
 *   dpad up / down     : target velocity +/- 50 ticks/sec
 *   dpad right / left  : target velocity +/- 500 ticks/sec
 *   Set kP to 0 first to see how close feedforward alone gets you.
 *
 * B : stop (power 0, target 0)
 *
 * Gains are public static with @Config, so you can change them live
 * in FTC Dashboard (192.168.43.1:8080/dash) and graph target vs. velocity.
 */
@Config
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends LinearOpMode {

    // ---- Tunable from FTC Dashboard ----
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kP = 0.0;
    public static double NOMINAL_VOLTAGE = 12.0;   // voltage the gains were tuned at

    // ---- Change these to match your robot config ----
    private static final String LEFT_NAME  = "flywheelLeft";
    private static final String RIGHT_NAME = "flywheelRight";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bulk caching: one hub read per loop instead of one per call
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        DcMotorEx left  = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Motors face each other across the shaft, so reverse one.
        // Flip which one if the wheel spins backward.
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        double voltage = battery.getVoltage();
        ElapsedTime voltageTimer = new ElapsedTime();

        boolean closedLoop = false;
        double openPower = 0.0;
        double target = 0.0;

        boolean prevUp = false, prevDown = false, prevLeft = false, prevRight = false;
        boolean prevA = false, prevB = false;

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.clearBulkCache();
            }

            // Voltage reads are slow - only sample a few times per second
            if (voltageTimer.milliseconds() > 250) {
                voltage = battery.getVoltage();
                voltageTimer.reset();
            }

            // ---- Button edge detection ----
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean lf = gamepad1.dpad_left, rt = gamepad1.dpad_right;
            boolean a = gamepad1.a, b = gamepad1.b;

            if (a && !prevA) closedLoop = !closedLoop;
            if (b && !prevB) { openPower = 0; target = 0; }

            if (closedLoop) {
                if (up && !prevUp)       target += 50;
                if (down && !prevDown)   target -= 50;
                if (rt && !prevRight)    target += 500;
                if (lf && !prevLeft)     target -= 500;
                target = Math.max(0, target);
            } else {
                if (up && !prevUp)       openPower += 0.01;
                if (down && !prevDown)   openPower -= 0.01;
                if (rt && !prevRight)    openPower += 0.10;
                if (lf && !prevLeft)     openPower -= 0.10;
                openPower = Range.clip(openPower, 0, 1);
            }

            prevUp = up; prevDown = down; prevLeft = lf; prevRight = rt;
            prevA = a; prevB = b;

            // ---- Measure (average both encoders) ----
            double velocity = (left.getVelocity() + right.getVelocity()) / 2.0;

            // ---- Compute power ----
            double power;
            double error = target - velocity;
            if (closedLoop) {
                double ff = (target > 0) ? kV * target + kS : 0.0;
                power = (ff + kP * error) * (NOMINAL_VOLTAGE / voltage);
            } else {
                power = openPower;
            }
            power = Range.clip(power, 0, 1);   // flywheel only spins one way

            left.setPower(power);
            right.setPower(power);

            // ---- Telemetry ----
            telemetry.addData("Mode", closedLoop ? "CLOSED-LOOP (A to switch)" : "OPEN-LOOP (A to switch)");
            telemetry.addData("Battery V", "%.2f", voltage);
            telemetry.addData("Power sent", "%.3f", power);
            telemetry.addData("velocity", "%.0f", velocity);
            telemetry.addData("left vel", "%.0f", left.getVelocity());
            telemetry.addData("right vel", "%.0f", right.getVelocity());

            if (closedLoop) {
                telemetry.addData("target", "%.0f", target);
                telemetry.addData("error", "%.0f", error);
            } else if (velocity > 50) {
                // Voltage-corrected estimate: what kV would be at NOMINAL_VOLTAGE
                double effectivePower = openPower * (voltage / NOMINAL_VOLTAGE);
                telemetry.addData("kV estimate", "%.6f", (effectivePower - kS) / velocity);
            }
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
    }
}
