class LedControl {
    private :
        void spiTransfer(uint8_t opcode, uint8_t data);            // Send out a single command to the device

    public:
        /* 
         * Create a new controller 
         * dataPin		pin on the Arduino where data gets shifted out
         * clockPin		pin for the clock
         * csPin		pin for selecting the device 
         */
        LedControl();

        /* 
         * Set all 8 Led's in a row to a new state
         * row	        row which is to be set (0..7)
         * value	each bit set to 1 will light up the corresponding Led.
         */
        void setRow(int row, uint8_t value);
};

