package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/29/2017.
 */
@Autonomous(name = "Shoot_Ramp_Red", group = "Beacon")
public class Shoot_Ramp_Red extends Gyro_Beacon {

    public void runOpMode() throws InterruptedException {

        init_gyro_beacon();

        CDI.setLED(0, true);           //Blue light On
        CDI.setLED(1, false);          //Red light OFF

        waitForStart();

        /*
         *******************************************************
         *****************  Main Control ***********************
         *******************************************************
         */

        go_forward(3, 0, -1, false, 0, false);
        turn_to_heading(325);
        go_forward(16, 325, -1, false, 0, false);
        turn_to_heading(330);
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();
        turn_to_heading(80);
        go_forward(43, 80, 1, false, 0, false);

    }
}