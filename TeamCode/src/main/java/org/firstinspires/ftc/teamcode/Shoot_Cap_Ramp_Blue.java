package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/18/2017.
 */
@Autonomous(name = "Shoot_Cap_Pirouette_Ramp_Blue", group = "Beacon")
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

        go_forward(3, 0, -1, false, 0, false);
        turn_to_heading(35);
        go_forward(16, 35, -1, false, 0, false);
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();
        turn_to_heading(62);  //75
        go_forward(33, 62, -1, false, 0, false); //75
        turn_to_heading_pirouette(280); //260
        turn_to_heading_pirouette(225); //260
        //turn_to_heading(15);
        //turn_to_heading(135);
        turn_to_heading(325);
        go_forward(39, 325, 1, false, 0, false);



    }
}
