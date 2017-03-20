/*
Gyro Beacon Autonomous Blue
by Drew Kinneer 1/25/2017
Team 10435
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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

abstract class Gyro_Beacon extends LinearOpMode {

    //hardware
    DeviceInterfaceModule CDI;
    private Servo ball_gate_servo;
    private DcMotor ShootMotor;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private I2cDeviceSynch ColorRightreader;  // right beacon sensor
    private I2cDeviceSynch ColorLeftreader;   // left beacon sensor
    private Servo btn_servo;
    private OpticalDistanceSensor ODS;
    private TouchSensor touchSensor;
    private ModernRoboticsI2cGyro gyro;

    private ElapsedTime ShootMotorRuntime = new ElapsedTime();

    // Output of the go_straight function
    boolean found_white = false;

    //Btn_Servo Variables
    private double init_btn_servo_position = .45;

    //Other
    private double gate_closed_position = .9;
    private double wheel_encoder_ticks = 1440;
    private double wheel_diameter = 3.62;  // size of Matrix wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * 3.1416);

    void init_gyro_beacon() {
        Servo fork_leveler;
        I2cDevice ColorRight;
        I2cDevice ColorLeft;
        int Passive = 1;
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        btn_servo = hardwareMap.servo.get("button_servo");
        touchSensor = hardwareMap.touchSensor.get("TouchSensor");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        fork_leveler = hardwareMap.servo.get("fork_leveler");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");


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

        fork_leveler.setPosition(.555);

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
    } // end of init_gyro_beacon


    void Shoot() {

        if (opModeIsActive()) {
            double open_position = 0;
            ShootMotor.setTargetPosition(ShootMotor.getCurrentPosition() + 2870);
            ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ShootMotor.setPower(1);
            ShootMotorRuntime.reset();
            while (opModeIsActive() && ShootMotorRuntime.seconds() < 2 && ShootMotor.isBusy()) {
                sleep(10);
                if (ShootMotorRuntime.seconds() > .4 && ShootMotorRuntime.seconds() < .8) {
                    ball_gate_servo.setPosition(open_position);  // open the gate for another ball
                } else if (ShootMotorRuntime.seconds() >= .8) {
                    ball_gate_servo.setPosition(gate_closed_position);  // close the gate
                }
            }
            ShootMotor.setPower(0);
            ball_gate_servo.setPosition(gate_closed_position);  // close the gate
            ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    } // end of Shoot


    void turn_to_heading(double target_heading) {
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

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel

        double gs_adjustment;
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
            gs_adjustment = 0;
        } else {
            gs_adjustment = (Math.pow((degrees_off + 2) / 3, 2) + 2) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        return gs_adjustment;

    } // end of go_straight_adjustment


    void go_forward(double inches_to_travel, int heading, double speed, boolean find_white, double inches_till_check, boolean use_touch_sensor) {

        DbgLog.msg("10435 starting go_forward inches:" + Double.toString(inches_to_travel) + " heading:" + Integer.toString(heading) + " speed:" + Double.toString(speed) + " find white:" + Boolean.toString(find_white) + " inches till check:" + Double.toString(inches_till_check) + " use touch sensor:" + Boolean.toString(use_touch_sensor));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        int ticks_till_check;
        boolean touch_sensor_pressed = false;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double white_value = .7;
        double white_level_read = 0;
        double speed_increase = .05;
        int start_position_L;
        int start_position_R;
        //int previous_ticks_traveled_L = 0;
        int ticks_traveled_L;
        int ticks_traveled_R;
        int lowest_ticks_traveled;
        double remaining_inches;
        //double previous_log_timer = 0;
        double power_adjustment;

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);
        ticks_till_check = (int) (inches_till_check * ticks_per_inch);

        start_position_L = leftWheel.getCurrentPosition();
        start_position_R = rightWheel.getCurrentPosition();

        found_white = false;

        log_timer.reset();

        telemetry.addData("go_forward ticks_to_travel", ticks_to_travel);

        while (opModeIsActive() && !destination_reached && !found_white && !touch_sensor_pressed) {

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);
            rightWheel.setPower(current_speed + power_adjustment);
            leftWheel.setPower(current_speed - power_adjustment);

            ticks_traveled_L = Math.abs(leftWheel.getCurrentPosition() - start_position_L);
            ticks_traveled_R = Math.abs(rightWheel.getCurrentPosition() - start_position_R);
            lowest_ticks_traveled = ticks_traveled_L;
            if (ticks_traveled_R < ticks_traveled_L) {
                lowest_ticks_traveled = ticks_traveled_R;
            }

            /*
            if (ticks_traveled_L != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 go_forward ticks_traveled: L:" + Double.toString(ticks_traveled_L)
                        + " R:" + Double.toString(ticks_traveled_R));

                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = ticks_traveled_L;
            }
            */

            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            if (find_white && lowest_ticks_traveled > ticks_till_check) {
                white_level_read = ODS.getLightDetected();
                found_white = white_level_read > white_value;
            }

            if (use_touch_sensor) {
                touch_sensor_pressed = touchSensor.isPressed();
            }

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= (10 * Math.abs(speed)) && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 go_forward slowing down: remaining_inches:" + Double.toString(remaining_inches)
                        + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }

        rightWheel.setPower(0);
        leftWheel.setPower(0);

        //telemetry.addData("go_forward destination_reached", destination_reached);
        //telemetry.addData("go_forward touch_sensor_pressed", touch_sensor_pressed);
        //telemetry.addData("go_forward found_white", found_white);
        //telemetry.update();

        sleep(100);
        DbgLog.msg("10435 ending go_forward: opModeIsActive:" + Boolean.toString(opModeIsActive())
                + " distance traveled L:" + Double.toString((leftWheel.getCurrentPosition() - start_position_L) / ticks_per_inch)
                + " distance traveled R:" + Double.toString((rightWheel.getCurrentPosition() - start_position_R) / ticks_per_inch)
                + " destination_reached:" + Boolean.toString(destination_reached)
                + " found_white:" + Boolean.toString(found_white) + " white_level:" + Double.toString(white_level_read)
                + " touch_sensor_pressed:" + Boolean.toString(touch_sensor_pressed));

    } // end of go_forward


    void button_push(String BorR) {

        int colorlevelRight;
        int colorlevelLeft;
        int tries_count = 0;
        byte[] TempByte;
        boolean button_pressed;
        double color_good = 8;
        int current_color = 0x07;
        double btn_servo_position;
        double btn_servo_degrees = .4;

        DbgLog.msg("10435 starting button_push");

        if (BorR.equals("blue")) {
            current_color = 0x07;
            color_good = 8;
        } else if (BorR.equals("red")) {
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

    void beacon_cleanup(String BorR) {

        double btn_servo_degrees = .4;
        double right_servo_position = init_btn_servo_position - btn_servo_degrees;
        double left_servo_position = init_btn_servo_position + btn_servo_degrees;
        String right_or_left;

        right_or_left = beacon_reads(BorR);

        if (right_or_left.equals("left")) {
            btn_servo.setPosition(left_servo_position);
        } else if (right_or_left.equals("right")) {
            btn_servo.setPosition(right_servo_position);
        }
        sleep(500);
        btn_servo.setPosition(init_btn_servo_position);
    }

    String beacon_reads(String BorR) {

        byte[] TempByte;
        int bluelevelRight;
        int bluelevelLeft;
        int redlevelRight;
        int redlevelLeft;
        int color_good_blue = 8;
        int color_good_red = 8;
        String push = "";

        // Do the first color reads
        TempByte = ColorRightreader.read(0x07, 1);
        bluelevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(0x07, 1);
        bluelevelLeft = TempByte[0];
        TempByte = ColorRightreader.read(0x05, 1);
        redlevelRight = TempByte[0];
        TempByte = ColorLeftreader.read(0x05, 1);
        redlevelLeft = TempByte[0];

        telemetry.addData("Red: Left", redlevelLeft);
        telemetry.addData("Red: Right", redlevelRight);
        telemetry.addData("Blue: Left", bluelevelLeft);
        telemetry.addData("Blue: Right", bluelevelRight);
        telemetry.update();

        if (BorR.equals("red") && !(redlevelLeft >= color_good_red && redlevelRight >= color_good_red)) {
            if (redlevelLeft >= color_good_red && bluelevelRight >= color_good_blue) {
                push = "left";
            } else if (redlevelRight >= color_good_red && bluelevelLeft >= color_good_blue) {
                push = "right";
            } else if (bluelevelLeft >= color_good_blue && bluelevelRight >= color_good_blue) {
                push = "left";
            }
        } else if (BorR.equals("blue") && !(bluelevelLeft >= color_good_blue && bluelevelRight >= color_good_blue)) {
            if (bluelevelLeft >= color_good_blue && redlevelRight >= color_good_red) {
                push = "left";
            } else if (bluelevelRight >= color_good_blue && redlevelLeft >= color_good_red) {
                push = "right";
            } else if (redlevelLeft >= color_good_red && redlevelRight >= color_good_red) {
                push = "right";
            }
        }
        return push;
    }

}





