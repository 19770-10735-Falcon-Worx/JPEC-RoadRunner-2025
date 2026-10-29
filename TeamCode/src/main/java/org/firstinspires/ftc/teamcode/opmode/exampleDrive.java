package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.FlowerSpintake;
import org.firstinspires.ftc.teamcode.mechanisms.Loader;
import org.firstinspires.ftc.teamcode.mechanisms.PIDConstants;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transition;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class exampleDrive extends OpMode {
    double[] encoderPositions = {0.0, 0.3333, 0.6667};
    ProgrammingBoard1 board = new ProgrammingBoard1();
    //FlowerSpintake flowerSpintake = new FlowerSpintake();

    boolean yPressed, aPressed, bPressed, shooterOn, downPressed, upPressed, RBPressed, LBPressed, xPressed, LTPressed, RTPressed;
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
    private Transition m_transition;
    private Shooter m_shooter;
    private Loader m_loader;
    double  shooterPercent = 1400;
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
        m_shooter = new Shooter(this.hardwareMap);
        m_loader = new Loader(this.hardwareMap);
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

        /* Intake control */

        if (gamepad2.y) board.setIntakePower(-1);
        else if (gamepad2.b) board.setIntakePower(1);
        else board.setIntakePower(0);

        // Shooter Control
        if (gamepad1.left_trigger > 0.25 && !LTPressed) shooterOn = !shooterOn;

        if (shooterOn) {
            m_shooter.setShooterSpeed(shooterPercent);
        } else {
            m_shooter.setShooterSpeed(0);
        }


        if (gamepad1.right_trigger > 0.25) {
            m_loader.setLoaderMotorPower(-0.5);
        } else {
            m_loader.setLoaderMotorPower(0);
        }



        // Spintake control
/*
        if(gamepad2.y){
            flowerSpintake.setArmPosition(0.0);
        }

        if(gamepad2.b){
            flowerSpintake.setArmPosition(1.0);
        }

        if(gamepad2.right_bumper){
            flowerSpintake.spinner(-1.0);
        }

        if(gamepad2.left_bumper){
            flowerSpintake.spinner(1.0);
        }
        */
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

        if (gamepad1.dpad_down && !downPressed) shooterPercent -= 50;
        if (gamepad1.dpad_up && !upPressed) shooterPercent += 50;

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
        telemetry.addData("Shooter Speed Percentage", shooterPercent);
        telemetry.addData("Shooter Current RPM", m_shooter.getShooterSpeed());
        telemetry.addData("Shooter On", shooterOn);
        PIDFCoefficients newPID = new PIDFCoefficients(PIDConstants.P_VALUE, PIDConstants.I_VALUE, PIDConstants.D_VALUE, PIDConstants.F_VALUE);
        m_shooter.setPIDF(newPID);
        PIDFCoefficients currentPID = m_shooter.getPIDValues();
        telemetry.addData("Current P", currentPID.p);
        telemetry.addData("Current I", currentPID.i);
        telemetry.addData("Current D", currentPID.d);
        telemetry.addData("Current f", currentPID.f);
        telemetry.addData("Version", "1.1");


    }
}
