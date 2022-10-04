package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

public enum BluePosition {
    //List of positions
    DUCKSIDESTART(new RobotPosition(new TwoDimensionalVector(0, 0), 0)),
    WAREHOUSESIDESTART(new RobotPosition(new TwoDimensionalVector(0, 0), 0));

    private RobotPosition position;

    private BluePosition(final RobotPosition position) {
        this.position = position;
    }

}
