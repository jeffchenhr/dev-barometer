#include <mex.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>

#include <termios.h>
#include <unistd.h>     // for read and return value of lseek
#include <sys/time.h>   // for select_time
extern int kbhit(void);
extern int getch(void);
extern struct termios keyboard_origtty;

#include <ctype.h>      // isdigit()
#include <stdio.h>      // stdout
#include <stdarg.h>
#include <time.h>
#include <fcntl.h>
#include <sys/errno.h>
#include <semaphore.h>


// fast serial port access from matlab for mac (written in osx 10.8.4)

int fdCom;
struct termios oldtio, newtio;
int debug_level = 10;
unsigned serial_timeout_count;

void DebugPrintf(int level, const char *fmt, ...)
{
    va_list ap;

    if (level <= debug_level)
    {
        char pTemp[2000];
        va_start(ap, fmt);
        //vprintf(fmt, ap);
        vsprintf(pTemp, fmt, ap);
        printf("%s", pTemp);
        va_end(ap);
        fflush(stdout);
    }
}

void closeSerial()
{
    tcflush(fdCom, TCOFLUSH);
    tcflush(fdCom, TCIFLUSH);
    tcsetattr(fdCom, TCSANOW, &oldtio);
    close(fdCom);
    DebugPrintf(3, "COM-Port closed.\n");
}

void openSerial(char *portname, int baud_rate)
{
    fdCom = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fdCom < 0)
    {
        int err = errno;
        DebugPrintf(1, "Can't open COM-Port %s ! (Error: %dd (0x%X))\n", portname, err, err);
        exit(2);
    }

    DebugPrintf(3, "COM-Port %s opened...\n", portname);

    mexAtExit(closeSerial);

    /* clear input & output buffers, then switch to "blocking mode" */
    tcflush(fdCom, TCOFLUSH);
    tcflush(fdCom, TCIFLUSH);
    fcntl(fdCom, F_SETFL, fcntl(fdCom, F_GETFL) & ~O_NONBLOCK);

    tcgetattr(fdCom, &oldtio); /* save current port settings */

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;

    newtio.c_ispeed = newtio.c_ospeed = baud_rate;

    newtio.c_iflag = IGNPAR | IGNBRK | IXON | IXOFF;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    cfmakeraw(&newtio);
    newtio.c_cc[VTIME]    = 1;   /* inter-character timer used */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 0 chars received */

    tcflush(fdCom, TCIFLUSH);
    if(tcsetattr(fdCom, TCSANOW, &newtio))
    {
       DebugPrintf(1, "Could not change serial port behaviour (wrong baudrate?)\n");
       exit(3);
    }
}

void ClearSerialPortBuffers()
{
    /* variables to store the current tty state, create a new one */
    struct termios origtty, tty;

    /* store the current tty settings */
    tcgetattr(fdCom, &origtty);

    // Flush input and output buffers
    tty=origtty;
    tcsetattr(fdCom, TCSAFLUSH, &tty);

    /* reset the tty to its original settings */
    tcsetattr(fdCom, TCSADRAIN, &origtty);
}

/***************************** SerialTimeoutSet *************************/
/**  Sets (or resets) the timeout to the timout period requested.  Starts
counting to this period.  This timeout support is a little odd in that the
timeout specifies the accumulated deadtime waiting to read not the total
time waiting to read. They should be close enought to the same for this
use. Used by the serial input routines, the actual counting takes place in
ReceiveComPortBlock.
\param [in] timeout_milliseconds the time in milliseconds to use for
timeout.  Note that just because it is set in milliseconds doesn't mean
that the granularity is that fine.  In many cases (particularly Linux) it
will be coarser.
*/
static void SerialTimeoutSet(unsigned timeout_milliseconds)
{
    serial_timeout_count = timeout_milliseconds / 100;
}



/***************************** SerialTimeoutCheck ***********************/
/**  Check to see if the serial timeout timer has run down.
\retval 1 if timer has run out.
\retval 0 if timer still has time left.
*/
static int SerialTimeoutCheck()
{
    if (serial_timeout_count == 0)
    {
        return 1;
    }
    return 0;
}

void Sleep(unsigned long MilliSeconds)
{
    usleep(MilliSeconds*1000); //convert to microseconds
}

/***************************** SerialTimeoutTick ************************/
/**  Performs a timer tick.  In this simple case all we do is count down
with protection against underflow and wrapping at the low end.
*/
static void SerialTimeoutTick()
{
    if (serial_timeout_count <= 1)
    {
        serial_timeout_count = 0;
    }
    else
    {
        serial_timeout_count--;
    }
}

/***************************** SendComPortBlock *************************/
/**  Sends a block of bytes out the opened com port.
\param [in] s block to send.
\param [in] n size of the block.
*/
void SendComPortBlock(const void *s, size_t n)
{
    write(fdCom, s, n);
}

/***************************** SendComPort ******************************/
/**  Sends a string out the opened com port.
\param [in] s string to send.
*/
void SendComPort(const char *s)
{
    SendComPortBlock(s, strlen(s));
}



/***************************** ReceiveComPortBlock **********************/
/**  Receives a buffer from the open com port. Returns all the characters
ready (waits for up to 'n' milliseconds before accepting that no more
characters are ready) or when the buffer is full. 'n' is system dependant,
see SerialTimeout routines.
\param [out] answer buffer to hold the bytes read from the serial port.
\param [in] max_size the size of buffer pointed to by answer.
\param [out] real_size pointer to a long that returns the amout of the
buffer that is actually used.
*/
static void ReceiveComPortBlock(void *answer, unsigned long max_size,
                                          unsigned long *real_size)
{
    *real_size = read(fdCom, answer, max_size);

    if (*real_size == 0)
    {
        SerialTimeoutTick();
    }
}

/***************************** ReceiveComPort ***************************/
/**  Receives a buffer from the open com port. Returns when the buffer is
filled, the numer of requested linefeeds has been received or the timeout
period has passed
\param [in] ISPEnvironment.
\param [out] Answer buffer to hold the bytes read from the serial port.
\param [in] MaxSize the size of buffer pointed to by Answer.
\param [out] RealSize pointer to a long that returns the amout of the
buffer that is actually used.
\param [in] WantedNr0x0A the maximum number of linefeeds to accept before
returning.
\param [in] timeOutMilliseconds the maximum amount of time to wait before
reading with an incomplete buffer.
*/
void ReceiveComPort(const char *Ans, unsigned long MaxSize,
                                    unsigned long *RealSize, unsigned long WantedNr0x0A,
                                    unsigned timeOutMilliseconds)
{
    unsigned long tmp_realsize;
    unsigned long nr_of_0x0A = 0;
    unsigned long nr_of_0x0D = 0;
    int eof = 0;
    unsigned long p;
    unsigned char *Answer;

    Answer = (unsigned char*) Ans;

    SerialTimeoutSet(timeOutMilliseconds);

    (*RealSize) = 0;

    do
    {
        ReceiveComPortBlock(Answer + (*RealSize), MaxSize - 1 - (*RealSize), &tmp_realsize);

        if (tmp_realsize != 0)
        {
            for (p = (*RealSize); p < (*RealSize) + tmp_realsize; p++)
            {
                if (Answer[p] == 0x0a)
                {
                    nr_of_0x0A++;
                }
                else if (Answer[p] == 0x0d)
                {
                    nr_of_0x0D++;
                }
                else if (((signed char) Answer[p]) < 0)
                {
                    eof = 1;
                }
            }
        }

        (*RealSize) += tmp_realsize;

    } while (((*RealSize) < MaxSize) && (SerialTimeoutCheck() == 0) && (nr_of_0x0A < WantedNr0x0A) && (nr_of_0x0D < WantedNr0x0A) && !eof);

    Answer[(*RealSize)] = 0;

}

/***************************** ReceiveComPortBlockComplete **************/
/**  Receives a fixed block from the open com port. Returns when the
block is completely filled or the timeout period has passed
\param [out] block buffer to hold the bytes read from the serial port.
\param [in] size the size of the buffer pointed to by block.
\param [in] timeOut the maximum amount of time to wait before guvung up on
completing the read.
\return 0 if successful, non-zero otherwise.
*/
int ReceiveComPortBlockComplete(void *block, size_t size, unsigned timeout)
{
    unsigned long realsize = 0, read;
    char *result;

    result = (char*) block;

    SerialTimeoutSet(timeout);

    do
    {
        ReceiveComPortBlock(result + realsize, size - realsize, &read);

        realsize += read;

    } while ((realsize < size) && (SerialTimeoutCheck() == 0));

    if (realsize != size)
    {
        return 1;
    }
    return 0;
}


#define PKT_SIZE (216)
#define PKT_COUNT 1024
#define RETURN_SIZE (64)
#define RETURN_SIZE_STATUS 8

sem_t mutex;
unsigned char buff[PKT_COUNT][PKT_SIZE];
int ptr_in = 0;
int ptr_out = 0;
int count = 0;
int count_trashed = 0;
int count_total = 0;
int total_bytes_read = 0;

int kill_tread = 0;

void *receiver(void *arg) {

    unsigned char answer[PKT_SIZE];
    sem_wait(&mutex);
    count = 0;
    ptr_in = 0;
    ptr_out = 0;
    sem_post(&mutex);
    
    while (!kill_tread) {

		if ((ReceiveComPortBlockComplete(answer, PKT_SIZE, 100) == 0) && (answer[PKT_SIZE-4] == 0x20) && (answer[PKT_SIZE-3] == 0x20) && (answer[PKT_SIZE-2] == 0x20) && (answer[PKT_SIZE-1] == 0x20)) {
            total_bytes_read += PKT_SIZE;

            /*int tick = *((int*)&answer[0]); // MSB 3 ... LSB 0
            DebugPrintf(1, "%6d %6d %6d %8d\n", count, ptr_in, ptr_out, tick);*/

            sem_wait(&mutex);
		    unsigned char *tmp_ptr = buff[ptr_in];
            memcpy(buff[ptr_in], answer, PKT_SIZE);
          
            if (count < PKT_COUNT) {
                count++;
    		    ptr_in = (ptr_in + 1) % PKT_COUNT;
            } else {
    		    ptr_in  = (ptr_in + 1) % PKT_COUNT;
    		    ptr_out = (ptr_out + 1) % PKT_COUNT;                
            }

            sem_post(&mutex);

     /*
            data[0] = tick;
            for (i = 0; i < 13; i++) {
				val = answer[2*i+5] * 256 + answer[2*i+4];
				if (val > 32767) val = val - 65536;
                data[i+1] = (double)val;
            }
            return max_size;*/
            
        } else {
        	int ok = 0;
        	while (ok < 4) {
	            //DebugPrintf(1, "\n%d", total_bytes_read);
                if (ReceiveComPortBlockComplete(answer, 1, 100) == 0)
                {
					total_bytes_read++;
	                //DebugPrintf(1, "\r%d", total_bytes_read);
                    //DebugPrintf(1, "X");
					if (answer[0] == 0x20) {
						ok++;
					} else {
						ok = 0;
					}
                }
        	}
        }
  }

  closeSerial();
  printf("serial_write_mac: THREAD EXITED!\n");
  kill_tread = 0;
  return NULL;
}


void mexFunction
    (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char str[1025];
    char *str_matlab;
    char *commandName;
    int baud = 57600;
    int debug = 0;
    double r = 0;
	mwSize mrows, ncols;
	double *y;
	double *z;
	int toread = 1;
	int imutoread = 10;

    if (nrhs < 1)
        mexErrMsgTxt("Usage: serial_write('open', 'COM40', baud);\n"
                           "\tserial_write('write', 'Hello');\n" 
                           "\tserial_write('write_line', ', World');\n"
                           "\tserial_write('write', 'test123', DEBUG);\n"
                           "\tserial_write('close')\n");

    commandName = (char *) mxArrayToString(prhs[0]);
    if ( strcmp(commandName, "open") == 0) {

        if (nrhs >= 3)
            baud = mxGetScalar(prhs[2]);

        if (nrhs >=2)
            str_matlab = (char *) mxArrayToString(prhs[1]);
        else
            mexErrMsgTxt("'open' requires at least one argument");

        openSerial(str_matlab, baud);
        ClearSerialPortBuffers();

        kill_tread = 0;

        pthread_t thread;
        int recv_port = 1;
        pthread_create(&thread, NULL, receiver, (void *) &recv_port);
        
        return;
    } else if (strcmp(commandName, "mwrite") == 0)
    {
        if (nrhs >= 2)
            y = mxGetPr(prhs[1]);
        else
            mexErrMsgTxt("'write' requires at least one argument");
		mrows = mxGetM(prhs[1]);
		ncols = mxGetN(prhs[1]);
		if (mrows != 1) {
			mexErrMsgTxt("Max 1 row.");
		}
		if (ncols > 64) {
			mexErrMsgTxt("Max 64 cols.");
		}
		writeSerialM(y, ncols, 0);
    } else if (strcmp(commandName, "setservos") == 0)
    {
        if (nrhs >= 2)
            y = mxGetPr(prhs[1]);
        else
            mexErrMsgTxt("'write' requires at least one argument");
		mrows = mxGetM(prhs[1]);
		ncols = mxGetN(prhs[1]);
		if (mrows != 1) {
			mexErrMsgTxt("Max 1 row.");
		}
		if (ncols > 64) {
			mexErrMsgTxt("Max 64 cols.");
		}
		writeSerialServos(y, ncols, 0);
    } else if (strcmp(commandName, "mread") == 0)
    {
        if (nrhs >= 2)
            toread = mxGetScalar(prhs[1]);
		plhs[0] = mxCreateDoubleMatrix(1,toread, mxREAL);
		plhs[1] = mxCreateDoubleMatrix(1,1, mxREAL);
		y = mxGetPr(plhs[0]);
		z = mxGetPr(plhs[1]);
		readSerialM(y, z, toread, 0);
    } else if (strcmp(commandName, "status") == 0)
    {
		plhs[0] = mxCreateDoubleMatrix(1,6, mxREAL);
		y = mxGetPr(plhs[0]);
        sem_wait(&mutex);
		y[0] = ptr_in;
        y[1] = ptr_out;
        y[2] = count;
        y[3] = count_trashed;
        y[4] = count_total;
        y[5] = total_bytes_read;
        sem_post(&mutex);
    } else if (strcmp(commandName, "readIMU") == 0)
    {
		plhs[0] = mxCreateDoubleMatrix(1,RETURN_SIZE, mxREAL);
		y = mxGetPr(plhs[0]);
		readSerialIMU(y,1);
    } else if (strcmp(commandName, "readIMUall") == 0)
    {
        sem_wait(&mutex);
        int ndim = count;
        sem_post(&mutex);

        plhs[0] = mxCreateDoubleMatrix(RETURN_SIZE, ndim, mxREAL);
		y = mxGetPr(plhs[0]);
        plhs[1] = mxCreateDoubleMatrix(RETURN_SIZE_STATUS, ndim, mxREAL);
		z = mxGetPr(plhs[1]);
		readSerialIMU(y, z, ndim);
    } else if (strcmp(commandName, "readwriteIMU") == 0)
    {
        if (nrhs >= 2)
            y = mxGetPr(prhs[1]);
        else
            mexErrMsgTxt("'write' requires at least one argument");
		mrows = mxGetM(prhs[1]);
		ncols = mxGetN(prhs[1]);
		if (mrows != 1) {
			mexErrMsgTxt("Max 1 row.");
		}
		if (ncols > 64) {
			mexErrMsgTxt("Max 64 cols.");
		}
		writeSerialServos(y, ncols, 0);

        plhs[0] = mxCreateDoubleMatrix(1,14, mxREAL);
		y = mxGetPr(plhs[0]);
		readSerialIMU(y,16);
    } else if ( (strcmp(commandName, "write") == 0) ||
                (strcmp(commandName, "write_line") == 0) )
    {
        if (nrhs >= 3)
            debug = mxGetScalar(prhs[2]);

        if (nrhs >= 2)
            str_matlab = (char *) mxArrayToString(prhs[1]);
        else
            mexErrMsgTxt("'write' requires at least one argument");

        if (strcmp(commandName, "write_line") == 0) {
            sprintf(str, "%s\r\n", str_matlab);
            r = writeSerial(str, debug);
        } else {
            r = writeSerial(str_matlab, debug);
        }
        plhs[0] = mxCreateDoubleScalar(r);
    } else if ( strcmp(commandName, "close") == 0) {
        kill_tread = 1;
        return;
    } else {
        mexErrMsgTxt("I don't understand");
    }

 }

int readSerialM(double *data, double *info, int size, int debug) {
	unsigned char dout[16];
	int i;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot write. Open serial port first");

    
    if (ReceiveComPortBlockComplete(dout, size, 100) == 0) {
        for (i = 0; i < size; i++) {
			data[i] = (double)dout[i]; 
		}
		info[0] = (double)size;
        return size;
    } else {
        char err_str[128];
        sprintf(err_str, "Could not read all %d bytes.", size);
        mexErrMsgTxt(err_str);
    }
    return 0;

}

int readSerialIMU(double *dataarray, double *statusarray, int ndim) {
	int i, val, idim;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot read. Open serial port first");

    sem_wait(&mutex);
    if (count >= ndim) {

        for (idim = 0; idim < ndim; idim++) {
            double * data   = &dataarray[idim * RETURN_SIZE];
            double * status = &statusarray[idim * RETURN_SIZE_STATUS];

            data[0] = *((int*)&buff[ptr_out][0]); //answer[3] * 16777216 + answer[2] * 65536 + answer[1] * 256 + answer[0];
            data[0] /= 1000.0;

            for (i = 0; i < 21; i++) { // 9 (mag, gyro, acc) + 6 (actuator) + 6 (radio)
                val = buff[ptr_out][2*i+5] * 256 + buff[ptr_out][2*i+4];
                if (val > 32767) val = val - 65536;
                data[i+1] = (double)val;
            }

            for (i = 0; i < 3+32; i++) { // 3 (rates) + 8*4 (quats)
                int tmp = *((int*)&buff[ptr_out][4*i+46]);
                data[i+22] = tmp;
            }
            
            for (i = 0; i < 6; i++) { // 3 (pos + setpoint)
                int tmp = *((int*)&buff[ptr_out][4*i+186]);
                data[i+57] = tmp;
            }

            int statval = buff[ptr_out][PKT_SIZE-5] * 256 + buff[ptr_out][PKT_SIZE-6];
            
            status[2]  = statval & 0x03; // radio
            status[1]  = (statval >>  2) & 0x03; // aligner
            status[0]  = (statval >>  4) & 0x01; // ahrs
            status[7]  = (statval >>  5) & 0x07; // arming
            status[3]  = (statval >>  8) & 0x01; // motors
            status[5]  = (statval >>  9) & 0x03; // vicon
            status[4]  = (statval >> 11) & 0x3f; // wrl
                        
            // normalize rates
            for (i = 0; i < 3; i++) {
                data[22 + i] = data[22 + i] / 4096.0;
            }
            // normalize quaternions
            for (i = 4; i < 32; i++) { // first 4 skipped bc it's commands
                data[25 + i] = data[25 + i] / 32768.0;
            }
            // normalize pos (mm)
            for (i = 0; i < 6; i++) {
                data[57 + i] = data[49 + i] / 1000.0;
            }
            
            ptr_out = (ptr_out + 1) % PKT_COUNT;
            count--;
        }
        
        sem_post(&mutex);
        return PKT_SIZE;
    } else {
        sem_post(&mutex);
        return 0;
    }
    return 0;
}

int writeSerialServos(double *data, int size, int debug) {
	char dout[64];
	int i, val;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot write. Open serial port first");

    dout[0] = 0;
    char crc = 0;
	for (i = 0; i < size; i++) {
		val = (int)data[i];
		dout[i+1] = val & 0xff; 
        crc -= dout[i+1];
	}
    dout[size+1] = crc;
    
    SendComPortBlock(dout, size+2);
    return size;
}

int writeSerialM(double *data, int size, int debug) {
	char dout[64];
	int i, val;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot write. Open serial port first");

	for (i = 0; i < size; i++) {
		val = (int)data[i];
		dout[i] = val & 0xff; 
	}

    SendComPortBlock(dout, size);
    return size;
}

int writeSerial(char *str, int debug) {
    if (fdCom == 0)
        mexErrMsgTxt("Cannot write. Open serial port first");

    if (debug)
        printf("Writing Serial: [%s]\n", str);

    SendComPort(str);
    
    return strlen(str);
}

