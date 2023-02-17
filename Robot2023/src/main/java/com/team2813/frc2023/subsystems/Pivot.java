package com.team2813.frc2023.subsystems;



public class Pivot extends Subsystem1d<Pivot.Rotations> {

    public enum Rotations implements Position {
        INTAKE(5.99),
        CUBE_OUTAKE(13.32),
        CONE_OUTAKE(13.32 ),
        RESET(0);
        // TODO: EXACT VALUES TBD (calculations might be off)

        @Override
        public double getPos() {
            return position;
        }

        final double position;

        Rotations(double position ) {
            this.position = position;
        }
    }


}
