package com.team2813.frc2023.util;

import com.team2813.frc2023.subsystems.Pivot;
import com.team2813.frc2023.subsystems.Wrist;

public enum NodeType {
    CONE(Wrist.Rotations.TOP_SCORE_CONE, Wrist.Rotations.MID_SCORE_CONE, Pivot.Rotations.MID_CONE),
    CUBE(Wrist.Rotations.TOP_SCORE_CUBE, Wrist.Rotations.MID_SCORE_CUBE, Pivot.Rotations.MID_CUBE);

    private final Wrist.Rotations scoringWristRotationsHigh;
    private final Wrist.Rotations scoringWristRotationsMid;
    private final Pivot.Rotations scoringPivotRotationsMid;

    NodeType(
            Wrist.Rotations scoringWristRotationsHigh,
            Wrist.Rotations scoringWristRotationsMid,
            Pivot.Rotations scoringPivotRotationsMid
    ) {
        this.scoringWristRotationsHigh = scoringWristRotationsHigh;
        this.scoringWristRotationsMid = scoringWristRotationsMid;
        this.scoringPivotRotationsMid = scoringPivotRotationsMid;
    }

    public Wrist.Rotations getScoringWristRotationsHigh() {
        return scoringWristRotationsHigh;
    }

    public Wrist.Rotations getScoringWristRotationsMid() {
        return scoringWristRotationsMid;
    }

    public Pivot.Rotations getScoringPivotRotationsMid() {
        return scoringPivotRotationsMid;
    }
}
