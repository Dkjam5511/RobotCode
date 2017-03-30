package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/29/2017.
 */
@Autonomous(name = "Shoot_Center_Blue", group = "Beacon")
public class Shoot_Center_Blue extends Gyro_Beacon {

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
        turn_to_heading(35);
        go_forward(16, 35, -1, false, 0, false);
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();
        turn_to_heading(30);
        go_forward(30, 30, -1, false, 0, false);

    }
}