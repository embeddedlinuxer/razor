/*************************************************/
/*  Buffers.c					 */
/*  Copyright 2018, Phase Dynamics Inc.		 */
/*************************************************/

#ifndef BUFFERS_H_
#define BUFFERS_H_

#define MAX_BFR_SIZE	(1024)
#define MAX_BFR_SIZE_F	(60)

// unsigned integer buffer type
typedef struct { //circular FIFO buffer
			int		head;
			int		tail;
			int		n;
			Uint16		gseed;
			Uint16		crc16;
			unsigned int	buff[MAX_BFR_SIZE];
		} BFR;

// double - floating point buffer type
typedef struct { //circular FIFO buffer
			int		head;
			int		tail;
			int		n;
			double		buff[MAX_BFR_SIZE_F];
		} FP_BFR;

#endif /* BUFFERS_H_ */
