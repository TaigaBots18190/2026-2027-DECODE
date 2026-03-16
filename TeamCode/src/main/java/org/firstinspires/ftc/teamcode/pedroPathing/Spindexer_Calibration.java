package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Spindexer Calibration")
public class Spindexer_Calibration extends LinearOpMode{

    ElapsedTime controller = new ElapsedTime();

    public void runOpMode() {

        Servo servo1 = hardwareMap.get(Servo.class, "index");
        Boolean lessThan05 = null;

        servo1.setPosition(0);
        double servoPos = 0;
        double increment = 0.2055;
        boolean activation = false;

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                servoPos += 0.0001;
            }
            if (gamepad1.left_bumper) {
                servoPos -= 0.0001;
            }

            servoPos = Math.max(0, Math.min(1, servoPos));

            servo1.setPosition(servoPos);

            double currentPos = servo1.getPosition();

            if (gamepad1.aWasPressed() && !activation) {
                if (servo1.getPosition() < 0.5) {
                    lessThan05 = true;
                } else {
                    lessThan05 = false;
                }
                try {
                    BufferedWriter writer = new BufferedWriter(new FileWriter(AppUtil.getInstance().getSettingsFile("calibrationInfo.txt")));
                    if (lessThan05) {
                        writer.write((currentPos) + "\n" + (currentPos+increment) + "\n" + (currentPos+increment*2) + "\n" + (currentPos+0.3121) + "\n" + (currentPos+0.3121+increment) + "\n" + (currentPos+0.3121+increment*2));
                    } else {
                        writer.write((currentPos) + "\n" + (currentPos-increment) + "\n" + (currentPos-increment*2) + "\n" + (currentPos-0.3121) + "\n" + (currentPos-0.3121-increment) + "\n" + (currentPos-0.3121-increment*2));
                    }
                    writer.close();
                    activation = true;
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!activation) {
                telemetry.addData("Servo Position", servo1.getPosition());
            } else {
                telemetry.addLine("Success!");
            }
            telemetry.update();


        }
    }
}
