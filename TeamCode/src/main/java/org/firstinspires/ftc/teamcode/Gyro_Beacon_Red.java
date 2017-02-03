/*
Gyro Beacon Autonomous Red
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Gyro_Beacon_Red", group = "Beacon")

public class Gyro_Beacon_Red extends LinearOpMode {

    //hardware
    Servo ball_gate_servo;
    DcMotor ShootMotor;
    DcMotor leftWheel;
    DcMotor rightWheel;
    I2cDevice ColorRight;
    I2cDeviceSynch ColorRightreader;  // right beacon sensor
    I2cDevice ColorLeft;
    I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    DeviceInterfaceModule CDI;
    Servo btn_servo;
    OpticalDistanceSensor ODS;
    TouchSensor touchSensor;
    ModernRoboticsI2cGyro gyro;

    private ElapsedTime runtime = new ElapsedTime();

    // Output of the go_straight function
    boolean found_white = false;

    //Btn_Servo Variables
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .4;

    //Other
    int Passive = 1;
    double gate_closed_position = .9;

    @Override
    public void runOpMode() throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        touchSensor = hardwareMap.touchSensor.get("TouchSensor");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        CDI.setLED(1, true);           //Red light On
        CDI.setLED(0, false);          //Blue light OFF

        // Set up right beacon sensor
        ColorRight = hardwareMap.i2cDevice.get("cs_right");
        ColorRightreader = new I2cDeviceSynchImpl(ColorRight, I2cAddr.create8bit(0x3c), false);
        ColorRightreader.engage();
        ColorRightreader.write8(3, Passive);    //Set the mode of the color sensor to passive

        // Set up left beacon sensor
        ColorLeft = hardwareMap.i2cDevice.get("cs_left");
        ColorLeftreader = new I2cDeviceSynchImpl(ColorLeft, I2cAddr.create8bit(0x3a), false);
        ColorLeftreader.engage();
        ColorLeftreader.write8(3, Passive);    //Set the mode of the color sensor to passive

        btn_servo.setPosition(init_btn_servo_position);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ball_gate_servo.setPosition(gate_closed_position);

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        /*
         *****************  Main Control ***********************
         */

        // get lined up
        go_forward(3, 0, .5, false, 0, false);
        turn_to_heading(325);

        // go to first white line
        go_forward(54, 325, 1, false, 0, false);
        go_forward(12, 325, .3, true, 0, false);
        if (!found_white) {
            turn_to_heading(0);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 0, -.3, true, 0, false);
        } else {
            //go_forward(2, 30, -.3, false, 0, false);
        }
        turn_to_heading(270);
        go_forward(14, 270, .5, false, 0, true);

        // hit first beacon
        button_push("red");

        // back up, get lined up
        go_forward(7, 270, -.5, false, 0, false);

        // shoot balls
        Shoot();
        sleep(1200);  // wait for next ball to roll in
        Shoot();

        // go to second white line
        turn_to_heading(0);
        go_forward(39, 0, 1, false, 0, false);
        go_forward(10, 0, .5, true, 0, false);

        if (!found_white) {
            turn_to_heading(15);  // If we missed the line, try to change angle before backing up.
            go_forward(14, 15, -.3, true, 0, false);
        } else {
            //go_forward(4.5, 0, -.3, false, 0, false);
        }
        turn_to_heading(270);
        go_forward(14, 270, .5, false, 0, true);

        // hit second beacon
        button_push("red");

        // back up
        go_forward(14, 270, -1, false, 0, false);

        // turn toward center
        turn_to_heading(140);
        go_forward(51, 140, 1, false, 0, false);

        DbgLog.msg("10435 done");

    } // end of RunOpMode


    private void Shoot() {

        if (opModeIsActive()) {
            double open_position = 0;
            ShootMotor.setTargetPosition(ShootMotor.getCurrentPosition() + 2640);
            ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ShootMotor.setPower(1);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2 && ShootMotor.isBusy()) {
                sleep(10);
                if (runtime.seconds() > .4 && runtime.seconds() < .8) {
                    ball_gate_servo.setPosition(open_position);  // open the gate for another ball
                } else if (runtime.seconds() >= .8) {
                    ball_gate_servo.setPosition(gate_closed_position);  // close the gate
                }
            }
            ShootMotor.setPower(0);
            ball_gate_servo.setPosition(gate_closed_position);  // close the gate
            ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    } // end of Shoot


    private void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;

        DbgLog.msg("10435 starting turn_to_heading");

        current_heading = gyro.getHeading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }
        while (degrees_to_turn > .5 && opModeIsActive()) {

            wheel_power = (10 * Math.pow((degrees_to_turn + 15) / 40, 3) + 10) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }

            rightWheel.setPower(wheel_power);
            leftWheel.setPower(-wheel_power);

            current_heading = gyro.getHeading();                                // get the new current reading
            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }
            //telemetry.addData("Wheel Power", wheel_power);
            //telemetry.addData("Degrees to Turn", degrees_to_turn);
            //telemetry.addData("Current Heading", current_heading);
            //telemetry.update();

        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        sleep(100);

        DbgLog.msg("10435 ending turn_to_heading");

    } // end of turn_to_heading


    private double go_straight_adjustment(int target_heading) {

        //  This function outputs global variables power_adjustment_R and power_adjustment_L

        double power_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        current_heading = gyro.getHeading();
        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < 1) {
            power_adjustment = 0;
        } else {
            power_adjustment = (Math.pow((degrees_off + 2) / 3, 2) + 2) / 100;
        }

        if (go_right) {
            power_adjustment = -power_adjustment;
        }

        return power_adjustment;

    } // end of go_straight_adjustment


    private void go_forward(double inches_to_travel, int heading, double speed, boolean find_white, int inches_till_check, boolean use_touch_sensor) {

        DbgLog.msg("10435 starting go_forward");

        double current_speed = .05;
        double ticks_to_travel;
        boolean touch_sensor_pressed = false;
        boolean destination_reached = false;
        double white_value = .8;
        double speed_increase = .05;
        int start_position_L;
        int start_position_R;
        int ticks_till_check;
        // Output of the go_straight_adjustment function
        double power_adjustment;

        ticks_to_travel = convert_inches_to_ticks(inches_to_travel);
        ticks_till_check = convert_inches_to_ticks(inches_till_check);

        start_position_L = leftWheel.getCurrentPosition();
        start_position_R = rightWheel.getCurrentPosition();

        found_white = false;

        if (speed < 0) {
            speed_increase = -speed_increase;
        }

        telemetry.addData("go_forward ticks_to_travel", ticks_to_travel);

        while (opModeIsActive() && !destination_reached && !found_white && !touch_sensor_pressed) {

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);
            rightWheel.setPower(current_speed + power_adjustment);
            leftWheel.setPower(current_speed - power_adjustment);

            if (find_white && leftWheel.getCurrentPosition() > ticks_till_check) {
                found_white = ODS.getLightDetected() > white_value;
            }
            if (use_touch_sensor) {
                touch_sensor_pressed = touchSensor.isPressed();
            }

            if (speed >= 0) {
                destination_reached =
                        ((leftWheel.getCurrentPosition() >= start_position_L + ticks_to_travel) ||
                                (rightWheel.getCurrentPosition() >= start_position_R + ticks_to_travel));
            } else {
                destination_reached =
                        ((leftWheel.getCurrentPosition() <= start_position_L - ticks_to_travel) ||
                                (rightWheel.getCurrentPosition() <= start_position_R - ticks_to_travel));

            }
        }

        rightWheel.setPower(0);
        leftWheel.setPower(0);

        //telemetry.addData("go_forward destination_reached", destination_reached);
        //telemetry.addData("go_forward touch_sensor_pressed", touch_sensor_pressed);
        //telemetry.addData("go_forward found_white", found_white);
        //telemetry.update();

        sleep(100);
        DbgLog.msg("10435 ending go_forward: opModeIsActive:" + Boolean.toString(opModeIsActive()) + " destination_reached:" + Boolean.toString(destination_reached) + " found_white:" + Boolean.toString(found_white) + " touch_sensor_pressed:" + Boolean.toString(touch_sensor_pressed));

    } // end of go_forward


    private void button_push(String BorR) {

        int colorlevelRight;
        int colorlevelLeft;
        int tries_count = 0;
        byte[] TempByte;
        boolean button_pressed;
        double color_good = 8;
        int current_color = 0x07;

        DbgLog.msg("10435 starting button_push");

        if (BorR == "blue") {
            current_color = 0x07;
            color_good = 8;
        } else if (BorR == "red") {
            current_color = 0x05;
            color_good = 4;
        }

        // Do the first color reads
        TempByte = ColorRightreader.read(current_color, 1);
        colorlevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(current_color, 1);
        colorlevelLeft = TempByte[0];

        // Button pressing section
        button_pressed = false;
        while (!button_pressed && opModeIsActive() && tries_count < 3) {

            if (colorlevelRight > colorlevelLeft && colorlevelRight >= color_good) {
                btn_servo_position = init_btn_servo_position - btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            } else if (colorlevelLeft > colorlevelRight && colorlevelLeft >= color_good) {
                btn_servo_position = init_btn_servo_position + btn_servo_degrees;
                btn_servo.setPosition(btn_servo_position);
            }
            sleep(1000);
            btn_servo.setPosition(init_btn_servo_position);
            tries_count = tries_count + 1;

            //Read color sensosrs
            TempByte = ColorRightreader.read(current_color, 1);
            colorlevelRight = TempByte[0];
            TempByte = ColorLeftreader.read(current_color, 1);
            colorlevelLeft = TempByte[0];

            button_pressed = colorlevelLeft >= color_good && colorlevelRight >= color_good;
        }

        sleep(300);
        DbgLog.msg("10435 ending button_push");

    }  // end of button_push


    private int convert_inches_to_ticks(double inches) {
        return (int) (inches / 11.39 * 1440);  // 11.39 is for matrix wheels which are 3.625 in diameter * Pi is 11.39
    }  // end of convert_inches_to_ticks

}



