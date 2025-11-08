package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;

@TeleOp
public class exampleDrive extends OpMode {
    double[] encoderPositions = {0.0, 0.3333, 0.6667};
    ProgrammingBoard1 board = new ProgrammingBoard1();
    boolean yPressed, shooterOn, rightTriggerPressed, leftTriggerPressed, downPressed, upPressed, rightBumperPressed, leftBumperPressed;
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
    int turntablePosition = 0;
    double servoKP = 10, targetRPM = 3000;
//    private OctoQuad m_OctoQuad;
//    private HuskyLens m_HuskyLens;

    @Override
    public void init() {
        board.init(hardwareMap);
        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = (ThreeDeadWheelLocalizer) m_drivetrain.localizer;
//        m_OctoQuad = hardwareMap.get(OctoQuad.class, "octoquad");
//        m_HuskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

//        m_HuskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void loop() {
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
        if (gamepad2.a) {
            board.setIntakePower(-1);
        } else if (gamepad2.b) {
            board.setIntakePower(1);
        } else {
            board.setIntakePower(0);
        }

        /* Shooter Control */
        if (gamepad2.y && !yPressed)
            shooterOn = !shooterOn;

        yPressed = gamepad2.y;
        if (shooterOn) {
            board.setShooterLeftSpeed(targetRPM);
            board.setShooterRightSpeed(targetRPM);
        } else {
            board.setShooterLeftSpeed(0);
            board.setShooterRightSpeed(0);
        }
        if (gamepad2.dpad_down && !downPressed)
            targetRPM -= 300;

        downPressed = gamepad2.dpad_down;
        if (gamepad2.dpad_up && !upPressed)
            targetRPM += 300;

        upPressed = gamepad2.dpad_up;

        /* Magazine control */
        if (gamepad2.right_bumper && !rightBumperPressed)
            turntablePosition++;

        rightBumperPressed = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !leftBumperPressed)
            turntablePosition--;

        leftBumperPressed = gamepad2.left_bumper;

        /* Shooting control */
        if (gamepad2.right_trigger > 0.25) {
            board.setTransitionMotorPower(1);

        } else if (gamepad2.left_trigger > 0.25) {
            board.setTransitionMotorPower(-1);

        } else {
            board.setTransitionMotorPower(0);
        }

        double servoPower = board.getServoError(encoderPositions[turntablePosition % 3]) * servoKP;
        telemetry.addData("servo power", servoPower);
        board.setServoPower(servoPower);

        telemetry.addData("Rotation", Math.toDegrees(heading.heading.toDouble()));
//        telemetry.addData("HuskyLens", m_HuskyLens.getConnectionInfo());
        telemetry.addData("Par0-Pos", m_localizer.par0.getPositionAndVelocity().position);
        telemetry.addData("Par1-Pos", m_localizer.par1.getPositionAndVelocity().position);
        telemetry.addData("perp-Pos", m_localizer.perp.getPositionAndVelocity().position);
        telemetry.addData("Shooter Speed Percentage", targetRPM / 60);
        telemetry.addData("Shooter On", shooterOn);
    }
}
