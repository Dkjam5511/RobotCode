/*
Gyro Beacon Autonomous Blue
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Beacon_Blue_Center", group = "Beacon")

public class Gyro_Beacon_Blue_Center extends Gyro_Beacon {

    public void runOpMode() throws InterruptedException {

        init_gyro_beacon();

        CDI.setLED(0, true);           //Blue light On
        CDI.setLED(1, false);          //Red light OFF

        waitForStart();

        gyro_beacon_blue_center();

    } // end of RunOpMode
}


