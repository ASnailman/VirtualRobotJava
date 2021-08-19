package org.firstinspires.ftc.teamcode.Practice;

public class PID {

    double P;
    double I;
    double D;
    double error;
    double sum_error;
    double prev_error;
    double Output;

    public double PID_Control(double Target, double kp, double ki, double kd, double Actual) {

        error = Target - Actual;
        sum_error = sum_error + error;
        P = kp * error;
        I = ki * sum_error;
        D = kd * (error - prev_error);
        Output = P + I + D;
        prev_error = error;

        return Output;
    }

}
