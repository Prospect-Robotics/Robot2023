package com.team2813.frc2023.util;

import com.team2813.frc2023.subsystems.Wrist;

public enum NodeType {
    CONE(Wrist.Rotations.TOP_SCORE_CONE), CUBE(Wrist.Rotations.TOP_SCORE_CUBE);

    private final Wrist.Rotations coneScoringWristRotations;

    NodeType(Wrist.Rotations coneScoringWristRotations) {
        this.coneScoringWristRotations = coneScoringWristRotations;
    }

    public Wrist.Rotations getConeScoringWristRotations() {
        return coneScoringWristRotations;
    }
}
