#include "encoder.h"
#include "serial_lnx.h"

namespace encoder_space {
	
	Encoder::Encoder(char *port, int baudRate, int argc, char **argv) : Sensor(argc, argv) {
		
		serialConnection = new Tserial();
        serialConnection->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
        usleep(100);
        
	}

    bool Encoder::connect(){
        return true;
    }

    bool Encoder::disconnect(){
        return true;
    }

    bool Encoder::fetch() {
        return true;
    }

    void Encoder::publish(int frame_rate) {
        return;
    }
	
	EncoderData Encoder::fetchEncoderData() {
		
		EncoderData returnValue;
		char input = ' ';
		
		returnValue.leftCount = 0;
		returnValue.rightCount = 0;
		
		do {
			input = serialConnection->getChar();
			if (input < 58 && input > 47) {
				returnValue.leftCount *= 10;
				returnValue.leftCount += input - 48;
			}
		} while (input != ' ');
		
		do {
			input = serialConnection->getChar();
			if (input < 58 && input > 47) {
				returnValue.rightCount *= 10;
				returnValue.rightCount += input - 48;
			}
		} while (input != ' ');
		
		return returnValue;
		
	}
	
}
