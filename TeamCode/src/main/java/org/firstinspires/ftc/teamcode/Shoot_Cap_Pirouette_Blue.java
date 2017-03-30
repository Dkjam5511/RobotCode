package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Drew on 3/18/2017.
 */
@Autonomous(name = "Shoot_Cap_Pirouette_Blue", group = "Beacon")
public class Shoot_Cap_Pirouette_Blue extends Gyro_Beacon {

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
        turn_to_heading(70);
        go_forward(32, 70, -1, false, 0, false);
        turn_to_heading_pirouette(280);
        turn_to_heading_pirouette(225);

    }
}
