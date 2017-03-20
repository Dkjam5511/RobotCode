package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/18/2017.
 */
@Autonomous(name = "Shoot_Cap_Ramp_Blue", group = "Beacon")
public class Shoot_Cap_Ramp_Blue extends Gyro_Beacon {

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

        go_forward(17, 0, -1, false, 0, false);
        turn_to_heading(45);
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();
        go_forward(30, 45, -1, false, 0, false);
        turn_to_heading(227);
        sleep(500);
        go_forward(36, 227, 1, false, 0, false);
        turn_to_heading(315);
        go_forward(26, 315, 1, true, 18, false);
        turn_to_heading(270);
        go_forward(8, 270, .5, false, 0, true);
        beacon_cleanup("blue");
        go_forward(6, 270, -.5, false, 0, false);
        turn_to_heading(45);
        go_forward(28, 45, 1, false, 0, false);
        turn_to_heading(315);
        go_forward(32, 315, 1, false, 0, false);


    }
}
