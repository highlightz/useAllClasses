// This file tests the validity of class CSyncSerialComm

#if 0

#include "syncSerialComm.h"

void genSend( float angle, float speed, char* send )
{
  // Frame header
	send[0] = 0xff;
	send[1] = 0xfe;

	// Speed control
	const int *p = (int *)&speed;
	send[5] = (*p>>24)&0xff;
	send[4] = (*p>>16)&0xff;
	send[3] = (*p>>8)&0xff;
	send[2] = (*p>>0)&0xff;

	// Angle control
	p = (int *)&angle;
	send[9] = (*p>>24)&0xff;
	send[8] = (*p>>16)&0xff;
	send[7] = (*p>>8)&0xff;
	send[6] = (*p>>0)&0xff;
}

void writeToPort( const char* send )
{
	CSyncSerialComm s0( "COM3" );
	s0.Open( );
	s0.ConfigPort( CBR_115200, 5 );
	s0.Write( send, 10 );
	s0.Close( );
}

int main(  )
{
  // Controls
	const float angle = 0.2f;
	const float speed = 0.1f;
	
	// Signal to be sent to serial port
	char send[10];
	
	genSend( angle, speed, send );
	
	writeToPort( send );
	
	return 0;
}

#endif
