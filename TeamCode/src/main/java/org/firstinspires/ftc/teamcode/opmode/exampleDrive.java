package org.firstinspires.ftc.teamcode.opmode;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.Loader;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter2026;
import org.firstinspires.ftc.teamcode.mechanisms.Transition;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class exampleDrive extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();

    boolean yPressed, aPressed, bPressed, shooterOn, downPressed, upPressed, RBPressed, LBPressed, xPressed, LTPressed, RTPressed;
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
    private Transition m_transition;
    private Shooter2026 m_shooter;
    private Loader m_loader;
    double  shooterRpm = 3300;
    static final double REVERSE_RPM = -1200; // slow reverse to clear jams without a current spike
    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        // TODO: Try commenting out 'board.init()' while testing m_shooter.
        //  I realized that we constantly call 'setShooterLeftSpeed' in the working code,
        //  so calling it constantly from a command should make no difference.

        // TODO: It should also be mentioned that you should comment out m_transition
        //  and m_shooter while they aren't in use.
        board.init(hardwareMap);
        //flowerSpintake.init(hardwareMap);

        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = (ThreeDeadWheelLocalizer) m_drivetrain.localizer;
        m_transition = new Transition(this.hardwareMap);
        m_loader = new Loader(this.hardwareMap);
        m_shooter = new Shooter2026(this.hardwareMap);
    }

    @Override
    public void loop() {
        /* Drivetrain Control */
        m_localizer.update();

        Vector2d input = new Vector2d(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x
        );

        double yaw = -m_drivetrain.lazyImu.get().getRobotYawPitchRollAngles().getYaw();
        Pose2d heading = new Pose2d(new Vector2d(0, 0), Math.toRadians(yaw));
        input = heading.heading.times(input);

        if (gamepad1.share) m_drivetrain.lazyImu.get().resetYaw();
        m_drivetrain.setDrivePowers(new PoseVelocity2d(input, gamepad1.right_stick_x));

        /* Intake control

        if (gamepad2.y) board.setIntakePower(-1);
        else if (gamepad2.b) board.setIntakePower(1);
        else board.setIntakePower(0); */

        // Shooter Control
        if (gamepad1.left_trigger > 0.25 && !LTPressed) shooterOn = !shooterOn;

        // One command per loop: reverse (held) overrides the on/off toggle.
        if (gamepad1.left_bumper) {
            m_shooter.setTargetRPM(REVERSE_RPM);    // eject / clear jam
        } else if (shooterOn) {
            m_shooter.setTargetRPM(shooterRpm);
        } else {

            m_shooter.setTargetRPM(0.0);
        }
        m_shooter.update();

/*        if (gamepad1.right_trigger > 0.25) {
            m_loader.setLoaderMotorPower(-0.5);
        } else if (gamepad1.right_bumper) {
            m_loader.setLoaderMotorPower(0.5);
        } else {
            m_loader.setLoaderMotorPower(0.0);
        } */

        // Loader: feed only when the flywheel is at speed. Reverse (unjam) is never blocked.
        boolean feedRequested = gamepad1.right_trigger > 0.25;
        boolean shooterReady = m_shooter.isReady();

        if (feedRequested && shooterReady) {
            m_loader.setLoaderMotorPower(-0.5);      // feed
        } else if (gamepad1.right_bumper) {
            m_loader.setLoaderMotorPower(0.5);       // reverse to clear a jam
        } else {
            m_loader.setLoaderMotorPower(0.0);
        }

        // TODO: Just like with a button, you can create a variable (shooterOnRan), and compare
        //  it to shooterOn; if shooterOn is true, but shooterOnRan isn't, run the loop.
        //  Make sure to update shooterOnRan to the same state as shooterOn in the 'Update Button Pressed' segment.
        //  If you take this approach, make sure 'else' only runs if shooterOn is false.
//        if (condition) {
            // TODO: Try adding a 'Supplier<Double> speed' argument to runShooter.
            //  You can then pass '() -> [wanted speed]' through.
            //  Use 'speed.get()' to get the target speed.
            //  Just to be clear, '[wanted speed]' should be replaced with something like 3500.0 or 800.0
//            runningActions.add(m_shooter.runShooter());
//        } else {
//            runningActions.add(m_shooter.stopShooter());
//        }

        if (gamepad1.dpad_down && !downPressed) shooterRpm -= 100;
        if (gamepad1.dpad_up && !upPressed) shooterRpm += 100;

        /* Magazine control */


        /* Shooting control */
        if (gamepad2.x) m_transition.setTransitionMotorPower(-1);
        else if (gamepad2.a) m_transition.setTransitionMotorPower(1);
        else m_transition.setTransitionMotorPower(0);

        //WIP: Run transition action



        /* Update Button Pressed */

        yPressed = gamepad1.y;
        xPressed = gamepad1.x;
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;

        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;


        RBPressed = gamepad1.right_bumper;
        LBPressed = gamepad1.left_bumper;
        LTPressed = gamepad1.left_trigger > 0.25;
        RTPressed = gamepad1.right_trigger > 0.25;

        /* Command Scheduling */
        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        /* Telemetry */
        dash.sendTelemetryPacket(packet); // Raw telemetry from command scheduling
        telemetry.addData("Rotation", Math.toDegrees(heading.heading.toDouble()));
        telemetry.addData("Shooter TARGET RPM", shooterRpm);
        telemetry.addData("Shooter ACTUAL RPM", m_shooter.getShooterRPM());
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Serial", android.os.Build.SERIAL);

    }

}
