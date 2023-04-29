#include "subsystems/subsystems.hpp"
#if USING_BEN_PNEUMATICS
    namespace pneumatics {
        namespace {
            class SinglePortPneumatics {
                private:
                    int output_port;
                    bool state1;
                    bool state2;
                    bool state3;
                    bool state4;

                    void update() {
                        double analog_value = 2;
                        if (this->state1)
                            analog_value += 63.5;
                        if (this->state2)
                            analog_value += 31.75;
                        if (this->state3)
                            analog_value += 15.875;
                        if (this->state4)
                            analog_value += 7.9375;
                        // set the analog value, check if it returns PROS_ERR
                        if (pros::c::adi_port_set_value(this->output_port, (int32_t)analog_value) ==
                            PROS_ERR) {
                            pros::lcd::print(1, "Error setting analog value");
                            printf("Error setting analog value: %d\n", errno);
                        }

                        printf("analog value: %f\n", analog_value);
                    }

                public:
                    SinglePortPneumatics(std::uint8_t adiport) {
                        this->output_port = adiport;
                        pros::c::adi_port_set_config(this->output_port, pros::E_ADI_LEGACY_PWM);
                        this->state1 = false;
                        this->state2 = false;
                        this->state3 = false;
                        this->state4 = false;
                    }

                    void set(std::uint8_t pneumatic, bool state) {
                        switch (pneumatic) {
                        case 1:
                            this->state1 = state;
                            break;
                        case 2:
                            this->state2 = state;
                            break;
                        case 3:
                            this->state3 = state;
                            break;
                        case 4:
                            this->state4 = state;
                            break;
                        }
                        for (int i = 0; i < 5; i++) {
                            this->update();
                            pros::delay(2);
                        }
                    }

                    bool get(std::uint8_t pneumatic) {
                        switch (pneumatic) {
                        case 1:
                            return this->state1;
                        case 2:
                            return this->state2;
                        case 3:
                            return this->state3;
                        case 4:
                            return this->state4;
                        }
                        return false;
                    }

                    /*
                        This is not recommended, as it will not update the state variables.
                        0-127 value;
                    */
                    void raw_write(std::uint32_t value) {
                        pros::c::adi_port_set_value(this->output_port, value);
                    }
            };


            SinglePortPneumatics pneumatics(PNEUMATICS_PORT);

            
        }

        void set_deflector(bool state) {
            pneumatics.set(DEFLECTOR, state);
        }

        void set_left_endgame(bool state) {
            pneumatics.set(LEFT_ENDGAME, state);
        }

        void set_right_endgame(bool state) {
            pneumatics.set(RIGHT_ENDGAME, state);
        }

        void set_blocker(bool state) {
            pneumatics.set(BLOCKER, state);
        }
    } // namespace pneumatics
#endif