package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class FlowerSpintake {
    public Servo arm;
    public CRServo spintake;
    double spinnerIdx = 0.0;

    public void init(HardwareMap hwMap){
        arm = hwMap.get(Servo.class, "arm");
        spintake = hwMap.get(CRServo.class, "spintake");
    }

    public void setArmPosition(double position){
        arm.setPosition(position);
    }

    public void setSpintakePower(double power){
        spintake.setPower(power);
    }

    public void spinner(double spinner){
        if(spinnerIdx == 0.0){
            if(spinner == -1.0){
                spintake.setPower(-1.0);
            }
            if(spinner == 1.0){
                spintake.setPower(1.0);
            }
        }

        if(spinnerIdx == -1.0){
            if(spinner == -1.0){
                spintake.setPower(0.0);
            }
            if(spinner == 1.0){
                spintake.setPower(1.0);
            }
        }
        if(spinnerIdx == 1.0){
            if(spinner == -1.0){
                spintake.setPower(1.0);
            }
            if(spinner == 1.0){
                spintake.setPower(0.0);
            }
        }
    }
}
