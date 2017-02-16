package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.internal.AppUtil;

/**
 * Created by Drew on 10/16/2016.
 * :P
 */
@TeleOp(name="TeleOp", group="Drive")
public class PushBotTeleOp extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor LiftMotor;
    DcMotor ShootMotor;
    DcMotor SweepMotor;
    Servo btn_servo;
    Servo left_lift_servo;
    Servo right_lift_servo;
    Servo ball_gate_servo;
    Servo ball_loader;
    DeviceInterfaceModule CDI;
    double init_btn_servo_position = .45;
    double btn_servo_position;
    double btn_servo_degrees = .2;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    double Speed = 1;
    double LiftSpeed = 1;
    double LeftSpeedInput;
    double RightSpeedInput;
    double dpad_speed = .143;
    double dpad_turn_speed = .16;
    double ServoPower;
    double LiftPower;
    double open_position = .4;
    double gate_closed_position = .9;
    double loader_down_position = .10;
    double loader_up_position = 1;
    double SM_start_position;
    boolean SlowMode = false;
    boolean BlueOn;
    boolean RedOn;
    boolean Reverse = false;
    boolean gate_open = false;
    boolean loader_up = false;
    boolean sweeper_running = false;
    boolean shoot_motor_running = false;


    private ElapsedTime gate_timer = new ElapsedTime();
    private ElapsedTime loader_timer = new ElapsedTime();

    @Override
    public void init() {
        //Setting Up Devices in the Hardware Map
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        LiftMotor = hardwareMap.dcMotor.get("lift_motor");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        SweepMotor = hardwareMap.dcMotor.get("sweep_motor");
        btn_servo = hardwareMap.servo.get("button_servo");
        left_lift_servo = hardwareMap.servo.get("left_fork");
        right_lift_servo = hardwareMap.servo.get("right_fork");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ball_loader = hardwareMap.servo.get("ball_loader");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        btn_servo.setPosition(init_btn_servo_position);
        right_lift_servo.setDirection(Servo.Direction.REVERSE);

        ServoPower = 0;

        ball_loader.setPosition(loader_down_position);

        ball_gate_servo.setPosition(gate_closed_position);

        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            SlowMode = false;
            BlueOn = false;
            RedOn = true;
        } else if (gamepad1.right_bumper) {
            SlowMode = true;
            BlueOn = true;
            RedOn = false;
        }

        if (SlowMode) {
            Speed = .25;
        } else {
            Speed = 1;
        }

        if (gamepad1.a) {
            Reverse = true;
        }

        if (gamepad1.b) {
            Reverse = false;
        }

        LeftSpeedInput = gamepad1.left_stick_y * Speed;
        RightSpeedInput = gamepad1.right_stick_y * Speed;

        if(gamepad1.dpad_up){
            LeftSpeedInput = -dpad_speed;
            RightSpeedInput = -dpad_speed;
        } else if (gamepad1.dpad_down){
            LeftSpeedInput = dpad_speed;
            RightSpeedInput = dpad_speed;
        } else if (gamepad1.dpad_left){
            LeftSpeedInput = dpad_turn_speed;
            RightSpeedInput = -dpad_turn_speed;
        } else if (gamepad1.dpad_right){
            LeftSpeedInput = -dpad_turn_speed;
            RightSpeedInput = dpad_turn_speed;
        }

        if (Reverse) {
            leftWheel.setDirection(DcMotor.Direction.REVERSE);
            rightWheel.setDirection(DcMotor.Direction.FORWARD);
            leftWheelPower = RightSpeedInput;
            rightWheelPower = LeftSpeedInput;
        } else {
            leftWheel.setDirection(DcMotor.Direction.FORWARD);
            rightWheel.setDirection(DcMotor.Direction.REVERSE);
            leftWheelPower = LeftSpeedInput;
            rightWheelPower = RightSpeedInput;
        }

        if(gamepad2.x){
            ShootMotor.setPower(1);
            shoot_motor_running = true;
        }

        if (shoot_motor_running && ShootMotor.getCurrentPosition() % 2880 > 2700) {
            ShootMotor.setPower(0);
            shoot_motor_running = false;
        }

        //Sending Those Wheel Powers to the Actual Wheels
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

       /* if (gamepad1.right_trigger == 1){
            btn_servo_position = init_btn_servo_position - btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }
        else if (gamepad1.right_trigger == 0){
            btn_servo_position = init_btn_servo_position;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.left_trigger == 1){
            btn_servo_position = init_btn_servo_position + btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }
        else if (gamepad1.left_trigger == 0){
            btn_servo_position = init_btn_servo_position;
            btn_servo.setPosition(btn_servo_position);
        }
        */

            //Setting the btn_servo position to the triggers

        if (gamepad1.right_trigger == 1) {
            btn_servo_position = init_btn_servo_position - btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.left_trigger == 1) {
            btn_servo_position = init_btn_servo_position + btn_servo_degrees;
            btn_servo.setPosition(btn_servo_position);
        }

        if (gamepad1.y) {
            btn_servo_position = init_btn_servo_position;
            btn_servo.setPosition(btn_servo_position);
        }


        if (gamepad2.right_bumper){
            LiftSpeed = .5;
        }

        if (gamepad2.left_bumper){
            LiftSpeed = 1;
        }


        ServoPower = gamepad2.right_stick_y / 2 + .5;
        right_lift_servo.setPosition(ServoPower);
        left_lift_servo.setPosition(ServoPower);


        LiftPower = gamepad2.left_stick_y * LiftSpeed;
        LiftMotor.setPower(LiftPower / 2);

        if (gamepad2.a) {
            gate_timer.reset();
            ball_gate_servo.setPosition(open_position);
            gate_open = true;
        }
        if (gate_open && gate_timer.milliseconds() > 180) {
            ball_gate_servo.setPosition(gate_closed_position);
            gate_open = false;
        }


        if (gamepad1.start) {
            SweepMotor.setPower(-1);
            ball_loader.setPosition(loader_down_position);
            loader_timer.reset();
            sweeper_running = true;
        }

        if (sweeper_running && loader_timer.seconds() <= 0.6 && !loader_up) {
            ball_loader.setPosition(loader_up_position);
            loader_up = true;
        } else if (sweeper_running && loader_timer.seconds() > 0.6 && loader_timer.seconds() < 2.1 && loader_up) {
            ball_loader.setPosition(loader_down_position);
            loader_up = false;
        } else if (loader_timer.seconds() >= 2.1) {
            loader_timer.reset();
        }

        if (gamepad1.x) {
            sweeper_running = false;
            SweepMotor.setPower(0);
            ball_loader.setPosition(loader_down_position);
        }

        telemetry.addData("Shoot Motor Position", ShootMotor.getCurrentPosition());

        /*
        if (gamepad2.b) {
            loader_timer.reset();
            ball_loader.setPosition(loader_up_position);
            loader_up = true;
        }
        if (loader_up && loader_timer.milliseconds() > 700) {
            ball_loader.setPosition(loader_down_position);
            loader_up = false;
        }
        */
        //Changing light based on speed
        CDI.setLED(0, BlueOn);           //Blue light
        CDI.setLED(1, RedOn);           //Red light


    }


    @Override
    public void stop() {

    }

}
