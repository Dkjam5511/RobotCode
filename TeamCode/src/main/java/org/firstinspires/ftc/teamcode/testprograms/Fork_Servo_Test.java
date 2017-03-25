package org.firstinspires.ftc.teamcode.testprograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Drew on 3/12/2017.
 */
@TeleOp(name = "Fork Servo Test", group = "Tests and Calibration")
public class Fork_Servo_Test extends OpMode {

    private ElapsedTime button_pressed = new ElapsedTime();

    Servo left_fork_servo;
    Servo right_fork_servo;
    Servo fork_leveler;

    double fork_position = .5;
    double leveler_position = .5;

    @Override
    public void init() {
        left_fork_servo = hardwareMap.servo.get("left_fork");
        right_fork_servo = hardwareMap.servo.get("right_fork");
        fork_leveler = hardwareMap.servo.get("fork_leveler");

        right_fork_servo.setDirection(Servo.Direction.REVERSE);

        left_fork_servo.setPosition(fork_position);
        right_fork_servo.setPosition(fork_position);
        fork_leveler.setPosition(leveler_position);
    }

    @Override
    public void loop() {
        if (button_pressed.seconds() > .2) {

            if (gamepad1.dpad_up) {
                button_pressed.reset();
                fork_position = fork_position + .01;
            }
            if (gamepad1.dpad_down) {
                button_pressed.reset();
                fork_position = fork_position - .01;
            }

            if (gamepad2.dpad_up) {
                button_pressed.reset();
                leveler_position = leveler_position + .01;
            }
            if (gamepad2.dpad_down) {
                button_pressed.reset();
                leveler_position = leveler_position - .01;
            }
        }

        left_fork_servo.setPosition(fork_position);
        right_fork_servo.setPosition(fork_position);
        fork_leveler.setPosition(leveler_position);

        telemetry.addData("Fork Position", fork_position);
        telemetry.addData("Leveler Position", leveler_position);
        telemetry.update();

    }
}
