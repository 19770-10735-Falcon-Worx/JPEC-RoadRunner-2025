package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Magazine {
    private final DcMotorEx magEncoder;
    private final CRServo magServo;

    public Magazine(HardwareMap hwMap) {
        magEncoder = hwMap.get(DcMotorEx.class, "intake"); // Mag uses intake encoder
        magEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        magServo = hwMap.get(CRServo.class, "turntable");
    }

    // TODO: This is where an actual Action over an InstantAction would be preferable.
    //  In the action, you want to constantly be running the following code (taken from exampleDrive):
    //  ---------------------------------------------------------------------------------------------
    //  private final double kServoKp = 5 // Move this line so the whole class can access it.
    //  private final double[] encoderPositions = {0.0, 0.3333, 0.6667}; // Same with this line.
    //  double servoError = board.getServoError(encoderPositions[turntablePosition % 3]);
    //  board.setServoPower(servoError * kServoKp);
    //  ---------------------------------------------------------------------------------------------
    //  until getServoError returns a relatively low value (+0.05 or -0.05 may be a good starting point)
    //  remember, returning false ends an action, returning true continues it.



    // TODO: Create a variable in this class, possibly called 'turntablePosition'.
    //  Increment or decrement it in an instant action, then run the action made based
    //  on my previous comment.

    public SequentialAction magSpin() {
        return new SequentialAction(

        );
    }

    private double getTurntablePosition() {
        if (magEncoder.getCurrentPosition() < 0)
            return 1 - Math.abs((magEncoder.getCurrentPosition() / 8192.0) % 1);
        return (magEncoder.getCurrentPosition() / 8192.0) % 1;
    }

    private void setServoPower(double power) {
        magServo.setPower(power);
    }

    private double getServoError(double target) {
        double servoError = target - getTurntablePosition();
        servoError = (servoError + 0.5) % 1;
        if (servoError < 0) {
            servoError += 1;
        }
        servoError -= 0.5;

        return servoError;
        //negative is because the servo isn't inverted
    }
}