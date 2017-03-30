/*
Gyro Beacon Autonomous Red
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Beacon_Red_Center", group = "Beacon")

public class Gyro_Beacon_Red_Center extends Gyro_Beacon {

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

        gyro_beacon_red_center();

    } // end of RunOpMode
}


