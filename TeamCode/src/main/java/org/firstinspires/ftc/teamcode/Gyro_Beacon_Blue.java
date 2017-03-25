/*
Gyro Beacon Autonomous Blue
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Gyro_Beacon_Blue", group = "Beacon")

public class Gyro_Beacon_Blue extends Gyro_Beacon {

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

        // get lined up
        go_forward(3, 0, .5, false, 0, false);
        turn_to_heading(32);

        // go to first white line
        go_forward(56, 32, 1, false, 0, false);
        go_forward(9, 32, .3, true, 0, false);
        if (!found_white) {
            turn_to_heading(0);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 0, -.3, true, 0, false);
        }

        turn_to_heading(89);
        go_forward(14, 90, .5, false, 0, true);

        // hit first beacon
        button_push("blue");

        go_forward(10, 98, -.7, false, 0, false);

        // shoot balls
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();

        // back up, get lined up
        go_forward(3, 98, .5, false, 0, false);

        // go to second white line
        turn_to_heading(0);
        go_forward(46, 0, 1, false, 0, false);
        go_forward(6, 0, .2, true, 0, false);

        if (!found_white) {
            turn_to_heading(345);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 345, -.3, true, 0, false);
        }

        turn_to_heading(89);
        go_forward(14, 90, .5, false, 0, true);

        // hit second beacon
        button_push("blue");

        // back up
        go_forward(14, 90, -1, false, 0, false);

        // turn toward center
        turn_to_heading(220);
        go_forward(49, 220, 1, false, 0, false);

        DbgLog.msg("10435 done");

    } // end of RunOpMode
}


