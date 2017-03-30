package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/18/2017.
 */
@Autonomous(name = "Shoot_Cap_Ramp_Red", group = "Beacon")
public class Shoot_Cap_Pirouette_Red extends Gyro_Beacon {

    public void runOpMode() throws InterruptedException {

        init_gyro_beacon();

        CDI.setLED(0, false);           //Blue light On
        CDI.setLED(1, true);          //Red light OFF

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
        turn_to_heading(290);  // 285
        go_forward(33, 290, -1, false, 0, false);  //285
        turn_to_heading_pirouette(80);  //100
        turn_to_heading_pirouette(135); //190

    }
}
