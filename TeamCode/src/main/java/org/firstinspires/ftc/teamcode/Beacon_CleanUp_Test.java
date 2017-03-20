package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/19/2017.
 */
@Autonomous(name = "Beacon_CleanUp_Test", group = "Tests and Calibration")
public class Beacon_CleanUp_Test extends Gyro_Beacon {
    public void runOpMode() throws InterruptedException {
        init_gyro_beacon();

        waitForStart();

        beacon_cleanup("blue");
        sleep(5000);
        beacon_cleanup("red");
        sleep(5000);
    }
}
