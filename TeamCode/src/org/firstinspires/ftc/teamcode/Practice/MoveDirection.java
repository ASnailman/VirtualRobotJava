package org.firstinspires.ftc.teamcode.Practice;

public enum MoveDirection {

    FORWARD(0), REVERSE(1), TOPDIAGRIGHT(2), TOPDIAGLEFT(3), BOTTOMDIAGRIGHT(4), BOTTOMDIAGLEFT(5), SIDEWAYSRIGHT(6), SIDEWAYSLEFT(7);

    int SetDirection;

    MoveDirection(int n) {

        SetDirection = n;

        }

    }
