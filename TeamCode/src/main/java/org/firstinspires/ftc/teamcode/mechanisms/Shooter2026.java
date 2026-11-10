package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Two-motor flywheel shooter with one shared feedforward + P controller.
 *
 *   power = (kV * target + kS + kP * error) * (NOMINAL_VOLTAGE / batteryVoltage)
 *
 * Gains are in ticks/sec units, the same as FlywheelTuner, so tuned values
 * copy straight in. The public API uses RPM like the old class.
 *
 * IMPORTANT: the controller only runs when update() is called.
 *   TeleOp: call shooter.update() once every loop.
 *   Auto:   run shooter.runShooter() inside a ParallelAction (it calls update()
 *           for you and finishes after stopShooter()).
 */
@Config
public class Shooter2026 {

    // ---- Gains: paste final values from the tuning guide ----
    public static double kS = 0.24;
    public static double kV = 0.000347;
    public static double kP = 0.008;
    public static double NOMINAL_VOLTAGE = 12.0;   // voltage the gains were tuned at

    // ---- Ready-to-shoot window ----
    public static double READY_TOLERANCE = 0.03;   // fraction of target (0.03 = 3%)
    public static double READY_TIME_MS = 100;      // must stay in tolerance this long

    public static double SHOOT_RPM = 3500;

    private static final double TICKS_PER_REV = 28.0;   // motor encoder, counted at the motor shaft

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final VoltageSensor battery;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private double voltage;
    private double targetTps = 0;     // target, ticks/sec
    private double velocityTps = 0;   // last measured, ticks/sec
    private double lastPower = 0;
    private boolean inTolerance = false;

    public Shooter2026(HardwareMap hwMap) {
        this(hwMap, "shooter", "shooter2");
    }

    public Shooter2026(HardwareMap hwMap, String leftName, String rightName) {
        left = hwMap.get(DcMotorEx.class, leftName);
        right = hwMap.get(DcMotorEx.class, rightName);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we run our own loop
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        // Motors face each other across the shaft, so one is reversed.
        setDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        battery = hwMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    /** Use if the wheel spins backward: swap which motor is reversed. */
    public void setDirections(DcMotorSimple.Direction leftDir, DcMotorSimple.Direction rightDir) {
        left.setDirection(leftDir);
        right.setDirection(rightDir);
    }

    // ---------------- Control loop ----------------

    /** Call once per loop. Reads velocity, computes power, sends it to both motors. */
    public void update() {
        // Voltage reads are slow; sample a few times per second
        if (voltageTimer.milliseconds() > 250) {
            double v = battery.getVoltage();
            if (v > 6) voltage = v;   // ignore bad/zero reads
            voltageTimer.reset();
        }

        velocityTps = (left.getVelocity() + right.getVelocity()) / 2.0;

        double power = 0;
        if (targetTps > 0) {
            double error = targetTps - velocityTps;
            power = (kV * targetTps + kS + kP * error) * (NOMINAL_VOLTAGE / voltage);
            power = Range.clip(power, 0, 1);   // flywheel only spins one way
        }
        left.setPower(power);
        right.setPower(power);
        lastPower = power;

        // Ready check: inside tolerance continuously for READY_TIME_MS
        boolean within = targetTps > 0
                && Math.abs(targetTps - velocityTps) <= READY_TOLERANCE * targetTps;
        if (!within) readyTimer.reset();
        inTolerance = within;
    }

    // ---------------- Speed API (RPM) ----------------

    public void setTargetRPM(double rpm) {
        targetTps = rpmToTps(Math.max(0, rpm));
        if (targetTps == 0) inTolerance = false;
    }

    public double getTargetRPM()  { return tpsToRpm(targetTps); }
    public double getShooterRPM() { return tpsToRpm(velocityTps); }
    public double getPower()      { return lastPower; }
    public double getVoltage()    { return voltage; }

    /** True once speed has stayed within READY_TOLERANCE for READY_TIME_MS. */
    public boolean isReady() {
        return inTolerance && readyTimer.milliseconds() >= READY_TIME_MS;
    }

    private static double rpmToTps(double rpm) { return rpm / 60.0 * TICKS_PER_REV; }
    private static double tpsToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }

    // ---------------- Road Runner actions ----------------

    /**
     * Spins up and holds speed. Keeps running (calling update()) until
     * stopShooter() sets the target to 0. Use inside a ParallelAction;
     * do NOT runBlocking() it by itself or it will never finish.
     */
    public Action runShooter(double rpm) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    setTargetRPM(rpm);
                    started = true;
                }
                update();
                packet.put("shooter target rpm", getTargetRPM());
                packet.put("shooter rpm", getShooterRPM());
                packet.put("shooter ready", isReady());
                return targetTps > 0;   // false = action done
            }
        };
    }

    public Action runShooter() {
        return runShooter(SHOOT_RPM);
    }

    /** Sets the target to 0; the running runShooter() action then stops the motors and ends. */
    public Action stopShooter() {
        return new InstantAction(() -> setTargetRPM(0));
    }

    /** Waits until the flywheel is at speed. Put this before feeding a ball. */
    public Action waitUntilReady() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return !isReady();
            }
        };
    }
}
