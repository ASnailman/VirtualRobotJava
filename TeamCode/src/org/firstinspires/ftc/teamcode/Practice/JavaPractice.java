package org.firstinspires.ftc.teamcode.Practice;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "JavaPractice", group = "mecanumbot")
public class JavaPractice extends LinearOpMode {

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DistanceSensor DistancesensorForward;
    static DistanceSensor DistancesensorRight;
    static ColorSensor Colorsensor;
    static MoveDirection Direction;
    static MoveDirection DiagDirection;
    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU ACCIMU;
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;
    double SteeringOutput;

    public void runOpMode () {

        MotorInitialize(
                hardwareMap.dcMotor.get("back_left_motor"),
                hardwareMap.dcMotor.get("back_right_motor"),
                hardwareMap.dcMotor.get("front_left_motor"),
                hardwareMap.dcMotor.get("front_right_motor")
        );

        SensorInitialize(
                hardwareMap.colorSensor.get("color_sensor")
        );

        SetDirection(MoveDirection.FORWARD);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        DistancesensorForward = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistancesensorRight = hardwareMap.get(DistanceSensor.class, "right_distance");
        IMU.initialize(parameters);

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            AutoGridpoint(5,5);

            //DirectionFollower2(20000, 0.6, 45, 0.0000000000000003, 0.00000000000000001, 0.00000000000005);
            //GyroTurn(0, 0.6);
            //DirectionFollower2(20000, 0.6, 1, 0.000000003, 0.000000000000001, 0.000000000005);
            //DirectionFollower2(20000, 0.5, 3, 0.0000003, 0.00000000000000001, 0.00000000001);
            //DirectionFollower2(20000, 0.5, 0, 0,0,0);
            //DiagonalMovement(MoveDirection.TOPDIAGLEFT, 0.8, 0.8, 2000);
            //DiagonalMovement(MoveDirection.TOPDIAGRIGHT, 1, 1, 5000);
            //DiagonalMovement(MoveDirection.SIDEWAYSRIGHT, 0.8, 0.8, 2000);
            //DiagonalMovement(MoveDirection.BOTTOMDIAGRIGHT, 0.8, 0.8, 1000);
            //DiagonalMovement(MoveDirection.BOTTOMDIAGLEFT, 0.8, 0.8, 2000);
            //DiagonalMovement(MoveDirection.SIDEWAYSLEFT, 0.8, 0.8, 4000);
            //WhiteDetector();
            //RedDetector();
            //return;
        }
    }

    private void MotorInitialize (DcMotor back_left_motor,
                                  DcMotor back_right_motor,
                                  DcMotor front_left_motor,
                                  DcMotor front_right_motor) {

        BackLeft = back_left_motor;
        BackRight = back_right_motor;
        FrontLeft = front_left_motor;
        FrontRight = front_right_motor;

    }

    private void SensorInitialize (ColorSensor color_sensor) {

        Colorsensor = color_sensor;

    }

    private void MotorTurn(double FR, double FL, double BR, double BL) {

        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        BackRight.setPower(BR);
        BackLeft.setPower(BL);

    }

    private void SetMotorPower(double x) {

        FrontLeft.setPower(x);
        FrontRight.setPower(x);
        BackLeft.setPower(x);
        BackRight.setPower(x);

    }

    private int WhiteDetector () {

        Colorsensor.red();
        Colorsensor.green();
        Colorsensor.blue();

        int White = 255;
        int Unknown = 1;

        telemetry.addData("Red", Colorsensor.red());
        telemetry.addData("Green", Colorsensor.green());
        telemetry.addData("Blue", Colorsensor.blue());

        if (Colorsensor.red() >= 190 && Colorsensor.green() >= 190 && Colorsensor.blue() >= 190) {

            telemetry.addData("Color:", "White");
            telemetry.update();
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            return White;

        } else {

            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            FrontLeft.setPower(0.8);
            FrontRight.setPower(0.8);
            BackLeft.setPower(0.8);
            BackRight.setPower(0.8);
            return Unknown;

        }
    }

    private int RedDetector () {

        Colorsensor.red();
        Colorsensor.green();
        Colorsensor.blue();

        int Red = 255;
        int Unknown = 1;

        telemetry.addData("Red", Colorsensor.red());
        telemetry.addData("Green", Colorsensor.green());
        telemetry.addData("Blue", Colorsensor.blue());

        if (Colorsensor.red() >= 190 && Colorsensor.green() <= 40 && Colorsensor.blue() <= 40) {

            telemetry.addData("Color:", "Red");
            telemetry.update();
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            return Red;

        } else {

            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            FrontLeft.setPower(1);
            FrontRight.setPower(-1);
            BackLeft.setPower(-1);
            BackRight.setPower(1);
            return Unknown;

        }
    }

    private void SetDirection (MoveDirection direction) {

        Direction = direction;

        if (Direction == MoveDirection.FORWARD) {
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
        } else if (Direction == MoveDirection.REVERSE) {
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private void DiagonalMovement (MoveDirection diagdirection, double Fpower, double Bpower, long sleep) {

        DiagDirection = diagdirection;
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (DiagDirection == MoveDirection.TOPDIAGRIGHT) {
            FrontLeft.setPower(Fpower);
            BackRight.setPower(Bpower);
            sleep(sleep);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
        } else if (DiagDirection == MoveDirection.TOPDIAGLEFT) {
            FrontRight.setPower(Fpower);
            BackLeft.setPower(Bpower);
            sleep(sleep);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        } else if (DiagDirection == MoveDirection.BOTTOMDIAGRIGHT) {
            FrontRight.setPower(-Fpower);
            BackLeft.setPower(-Bpower);
            sleep(sleep);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        } else if (DiagDirection == MoveDirection.BOTTOMDIAGLEFT) {
            FrontLeft.setPower(-Fpower);
            BackRight.setPower(-Bpower);
            sleep(sleep);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
        } else if (DiagDirection == MoveDirection.SIDEWAYSRIGHT) {
            FrontLeft.setPower(Fpower);
            FrontRight.setPower(-Fpower);
            BackLeft.setPower(-Bpower);
            BackRight.setPower(Bpower);
            sleep(sleep);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        } else if (DiagDirection == MoveDirection.SIDEWAYSLEFT) {
            FrontLeft.setPower(-Fpower);
            FrontRight.setPower(Fpower);
            BackLeft.setPower(Bpower);
            BackRight.setPower(-Bpower);
            sleep(sleep);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        }
    }

    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;

    }

    private void GyroTurn (double angledegree, double power) {

        SetDirection(MoveDirection.FORWARD);

        if (angledegree > GyroContinuity()) {

            while (GyroContinuity() <= angledegree) {

                MotorTurn(-power, power, -power, power);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (power > 0.5) {
                while (GyroContinuity() >= angledegree) {

                    MotorTurn(0.5, -0.5, 0.5, -0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }

        } else {

            while (GyroContinuity() >= angledegree) {

                MotorTurn(power, -power, power, -power);

                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();

            }

            // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
            if (power > 0.5) {
                while (GyroContinuity() <= angledegree) {

                    MotorTurn(-0.5, 0.5, -0.5, 0.5);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }
            }
        }
        SetMotorPower(0);
        sleep(200);
    }

    private void DirectionFollower2(double targetdistance, double power, double TargetDirection,
                                    double kp_in, double ki_in, double kd_in) {

        PID PID = new PID();
        FRpower = power;
        FLpower = power;
        BRpower = power;
        BLpower = power;

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (FrontRight.getCurrentPosition() < targetdistance) {

            SetDirection(MoveDirection.FORWARD);

            //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
            //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            FLpower = FLpower + SteeringOutput * FLpower;
            FRpower = FRpower - SteeringOutput * FRpower;
            BLpower = BLpower + SteeringOutput * BLpower;
            BRpower = BRpower - SteeringOutput * BRpower;

            FrontLeft.setPower(FLpower);
            FrontRight.setPower(FRpower);
            BackLeft.setPower(BLpower);
            BackRight.setPower(BRpower);

            telemetry.addData("Error", PID.error);
            telemetry.addData("TargetDirection", TargetDirection);
            telemetry.addData("Encoder", FrontRight.getCurrentPosition());
            telemetry.addData("Front_Left_Motor_Power", FLpower);
            telemetry.addData("Front_Right_Motor_Power", FRpower);
            telemetry.addData("Back_Left_Motor_Power", BLpower);
            telemetry.addData("Back_Right_Motor_Power", BRpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        while (-FrontRight.getCurrentPosition() > targetdistance) {

            SetDirection(MoveDirection.REVERSE);

            SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
            FLpower = FLpower - SteeringOutput * FLpower;
            FRpower = FRpower + SteeringOutput * FRpower;
            BLpower = BLpower - SteeringOutput * BLpower;
            BRpower = BRpower + SteeringOutput * BRpower;

            FrontLeft.setPower(FLpower);
            FrontRight.setPower(FRpower);
            BackLeft.setPower(BLpower);
            BackRight.setPower(BRpower);

            telemetry.addData("Encoder", -FrontRight.getCurrentPosition());
            telemetry.addData("Front_Left_Motor_Power", FLpower);
            telemetry.addData("Front_Right_Motor_Power", FRpower);
            telemetry.addData("Back_Left_Motor_Power", BLpower);
            telemetry.addData("Back_Right_Motor_Power", BRpower);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        //sleep(100);
        GyroTurn(TargetDirection, 0.3);

    }

    private void WallDetectorForwards (int Distance) {


        while (DistancesensorForward.getDistance(DistanceUnit.INCH) > Distance) {

            if (DistancesensorForward.getDistance(DistanceUnit.INCH) > Distance) {

                SetMotorPower(0.5);
                telemetry.addData("Distance", DistancesensorForward.getDistance(DistanceUnit.INCH));
                telemetry.update();

            } else {

                SetMotorPower(0);

            }
        }
    }

    private void WallDetectorCoordinateY (int y) {

        while (DistancesensorForward.getDistance(DistanceUnit.INCH) > y) {

            if (DistancesensorForward.getDistance(DistanceUnit.INCH) > y) {

                SetMotorPower(0.5);
                telemetry.addData("Distance", DistancesensorForward.getDistance(DistanceUnit.INCH));
                telemetry.update();

            } else {

                SetMotorPower(0);

            }
        }
    }



    private void WallDetectorCoordinateX (int x) {

        if (DistancesensorRight.getDistance(DistanceUnit.INCH) > x) {

            MotorTurn(-0.5, 0.5, 0.5, -0.5);
            telemetry.addData("Distance", DistancesensorRight.getDistance(DistanceUnit.INCH));
            telemetry.update();

        } else {

            SetMotorPower(0);

        }
    }

    private void AutoGridpoint (int x, int y) {

        WallDetectorCoordinateX(x);
        WallDetectorCoordinateY(y);

    }

}

