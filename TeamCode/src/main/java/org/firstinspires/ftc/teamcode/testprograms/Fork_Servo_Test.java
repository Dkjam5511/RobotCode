package org.firstinspires.ftc.teamcode.testprograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Drew on 3/12/2017.
 */
@TeleOp(name = "Fork Servo Test", group = "Tests and Calibration")
public class Fork_Servo_Test extends OpMode{

    Servo left_fork_servo;
    Servo right_fork_servo;


    @Override
    public void init() {
        left_fork_servo = hardwareMap.servo.get("left_fork");
        right_fork_servo = hardwareMap.servo.get("right_fork");

        right_fork_servo.setDirection(Servo.Direction.REVERSE);

        left_fork_servo.setPosition(.52);
        right_fork_servo.setPosition(.51);
    }

    @Override
    public void loop() {

    }
}
