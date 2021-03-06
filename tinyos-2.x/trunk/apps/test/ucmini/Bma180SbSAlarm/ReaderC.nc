#include "Bma180.h"
module ReaderC @safe()
{
  uses {
    interface Boot;
    interface Read<bma180_data_t>;
    interface ReadStream<bma180_data_t>;
    interface StdControl as BmaControl;
    interface Leds;
    interface AMSend;
    interface SplitControl as AMControl;
    interface DiagMsg;
  }
}
implementation
{
  enum
	{
		BUFFER_COUNT = 3,
		BUFFER_SIZE = 10,
		SAMPLING = BMA_SAMPLING_TIME_MS*1000,
	};

  bma180_data_t buffers[BUFFER_COUNT][BUFFER_SIZE];
 

  event void AMControl.startDone(error_t error) {
    uint8_t i;
    call BmaControl.start();
//    call Read.read();
    



		for(i = 0; i < BUFFER_COUNT; ++i)
		{
			error = call ReadStream.postBuffer(buffers[i], BUFFER_SIZE);
		}

		error = call ReadStream.read(SAMPLING);

  }

  event void Boot.booted() {
    //DDRF = _BV(PF0);
    //PORTF = _BV(PF0);
    
    call AMControl.start();
  }

  event void AMSend.sendDone(message_t* msg, error_t error) {}
  event void AMControl.stopDone(error_t error) {}
  event void Read.readDone(error_t result, bma180_data_t data) {
    if(call DiagMsg.record()) {
      call DiagMsg.str("Id: "); call DiagMsg.hex8(data.chip_id);
      call DiagMsg.str("X: ");
      call DiagMsg.int16( ((data.bma180_accel_x>>2) & 0x2000)?( -0.25*((~(data.bma180_accel_x-1))>>2) ):(data.bma180_accel_x>>2)*0.25);
      call DiagMsg.int16( ((int16_t)data.bma180_accel_x)>>2 );
      call DiagMsg.str("Y: ");
      call DiagMsg.int16( ((data.bma180_accel_y>>2) & 0x2000)?( -0.25*((~(data.bma180_accel_y-1))>>2) ):(data.bma180_accel_y>>2)*0.25);
      call DiagMsg.str("Z: ");
      call DiagMsg.int16( ((data.bma180_accel_z>>2) & 0x2000)?( -0.25*((~(data.bma180_accel_z-1))>>2) ):(data.bma180_accel_z>>2)*0.25);
      call DiagMsg.str("Temp: ");
      call DiagMsg.uint8(data.bma180_temperature);
      call DiagMsg.send();
    } //call Read.read();
  }

  event void ReadStream.bufferDone(error_t result, bma180_data_t* buf, uint16_t count) {
    //uint8_t i;
		//uint8_t *p;

		if( result == SUCCESS )
		{
			/*sampleCount += BUFFER_SIZE;

			p = call AMSend.getPayload(&dataMsg, BUFFER_SIZE);
			if( p != NULL )
			{
				for(i = 0; i < BUFFER_SIZE; ++i)
					p[i] = buf[i] >> 2;

				result = call AMSend.send(AM_BROADCAST_ADDR, &dataMsg, BUFFER_SIZE);
				if( result != SUCCESS )
					call Leds.led0Toggle();
			}*/

			//call ReadStream.postBuffer(buf, count);
		}
  }


  event void ReadStream.readDone(error_t err, uint32_t usperiod) {
    uint8_t cycle;
    if(call DiagMsg.record()) {
      for(cycle=0;cycle<10;cycle++)
        call DiagMsg.uint16(((buffers[2][cycle]).bma180_accel_z>>2)*0.25);
      call DiagMsg.send();
    }
  }
}
