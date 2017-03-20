/*
Gyro Beacon Autonomous Red
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Gyro_Beacon_Red", group = "Beacon")

public class Gyro_Beacon_Red extends Gyro_Beacon {

    public void runOpMode() throws InterruptedException {

        init_gyro_beacon();

        CDI.setLED(1, true);           //Red light On
        CDI.setLED(0, false);          //Blue light OFF

        waitForStart();

        /*
         *******************************************************
         *****************  Main Control ***********************
         *******************************************************
         */

        // get lined up
        go_forward(3, 0, .5, false, 0, false);
        turn_to_heading(328);

        // go to first white line
        go_forward(54, 328, 1, false, 0, false);
        go_forward(12, 328, .35, true, 0, false);
        if (!found_white) {
            turn_to_heading(0);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 0, -.3, true, 0, false);
        }

        turn_to_heading(270);
        go_forward(14, 270, .5, false, 0, true);

        // hit first beacon
        button_push("red");

        // back up, line up
        go_forward(10, 275, -.7, false, 0, false);

        // shoot balls
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();

        // goo forawrd, get lined up
        go_forward(3, 275, .5, false, 0, false);

        // go to second white line
        turn_to_heading(0);
        go_forward(39, 0, 1, false, 0, false);
        go_forward(10, 0, .2, true, 0, false);

        if (!found_white) {
            turn_to_heading(15);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 15, -.35, true, 0, false);
        }

        turn_to_heading(315);
        turn_to_heading(273);
        go_forward(14, 270, .5, false, 0, true);

        // hit second beacon
        button_push("red");

        // back up
        go_forward(14, 270, -1, false, 0, false);

        // turn toward center
        turn_to_heading(140);
        go_forward(49, 140, 1, false, 0, false);

        DbgLog.msg("10435 done");

    } // end of RunOpMode
}


