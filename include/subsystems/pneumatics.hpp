#pragma once
#if USING_BEN_PNEUMATICS
    namespace pneumatics {
        void set_deflector(bool state);
        void set_left_endgame(bool state);
        void set_right_endgame(bool state);
        void set_blocker(bool state);
    }
#endif