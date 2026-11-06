package org.firstinspires.ftc.teamcode.mechanisms;
import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {
    public static double P_VALUE = 2;
    public static double I_VALUE = 0;
    public static double D_VALUE = 0;
    public static double F_VALUE = 11.7; // 32767 / max ticks/sec at full power — re-measure for your flywheel
}
