package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class SharedClass {

    public static String motif = "PPG";
    public static double xPos = 7.5;
    public static double yPos = 7.8;
    public static double yaw = 90;

    public static double pos1Shoot;
    public static double pos2Shoot;
    public static double pos3Shoot;

    public static double pos1Intake;
    public static double pos2Intake;
    public static double pos3Intake;

    static {

        try {

            File file = AppUtil.getInstance().getSettingsFile("calibrationInfo.txt");
            BufferedReader reader = new BufferedReader(new FileReader(file));

            pos1Shoot = Double.parseDouble(reader.readLine());
            pos2Shoot = Double.parseDouble(reader.readLine());
            pos3Shoot = Double.parseDouble(reader.readLine());
            pos1Intake = Double.parseDouble(reader.readLine());
            pos2Intake = Double.parseDouble(reader.readLine());
            pos3Intake = Double.parseDouble(reader.readLine());

            reader.close();

        } catch (IOException e) {
            throw new RuntimeException("Failed to load calibration file", e);
        }
    }
}