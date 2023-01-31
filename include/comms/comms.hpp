#include "main.h"
#include "pros/apix.h"
#include <map>
namespace comms {
    class ReceiveComms {
        public: 
            ReceiveComms(uint8_t port, int baud, uint8_t start_char, uint8_t end_char);
            uint64_t get_data(char name);
            void start();
            void pause();

        private:
            pros::Mutex data_m;
            uint8_t byte_buf[7];
            std::shared_ptr<pros::Serial> ser;
            std::map<char, uint64_t> data;
            std::shared_ptr<pros::Task> data_task = nullptr;
            char sc, ec;
            uint8_t bytecount = 0;
            uint64_t tmp_storage = 0;
            void receive_task();
            void process_byte();
            char current_modify;
            bool modifying = false;
            bool select_char = false;
    };
}