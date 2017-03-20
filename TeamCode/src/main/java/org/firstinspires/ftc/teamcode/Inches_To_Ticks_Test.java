package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/18/2017.
 */
@Autonomous(name = "Inches_To_Ticks_Test", group = "Tests and Calibration")
public class Inches_To_Ticks_Test extends Gyro_Beacon {

    public void runOpMode() throws InterruptedException {

        init_gyro_beacon();

        waitForStart();

        go_forward(12, 0, -.3, false, 0, false);
        sleep(3000);
        go_forward(12, 0, .3, false, 0, false);
        sleep(3000);
        go_forward(144, 0, -1, false, 0, false);
        sleep(3000);
        go_forward(144, 0, 1, false, 0, false);
    }
}
