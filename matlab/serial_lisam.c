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

int openSerial(char *portname, int baud_rate)
{
    fdCom = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fdCom < 0)
    {
        int err = errno;
        DebugPrintf(1, "Can't open COM-Port %s ! (Error: %dd (0x%X))\n", portname, err, err);
        return 2;
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
       return 3;
    }
    
    return 0;
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

static unsigned int crc32_table[256];

void crc32_init() {
	unsigned int rem;
    int i,j;
	/* Calculate CRC table. */
	for (i = 0; i < 256; i++) {
		rem = i;  /* remainder from polynomial division */
		for (j = 0; j < 8; j++) {
			if (rem & 1) {
				rem >>= 1;
				rem ^= 0xedb88320;
			} else
				rem >>= 1;
		}
		crc32_table[i] = rem;
	}
}

unsigned int crc32_compute(unsigned int crc, const char *buf, size_t len)
{
	unsigned char octet;
	const char *p, *q;

	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc >> 8) ^ crc32_table[(crc & 0xff) ^ octet];
	}
	return ~crc;
}

#define PKT_SIZE (216+24+32+24+16+2)
#define PKT_COUNT 512
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
            DebugPrintf(1, "\n%d", total_bytes_read);
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


const char *fieldnames[] = { "tick", "mag", "gyro", "acc", "actuators",
                             "body_quat", "goal_quat", 
                             "status",
                             "radio", "rates",
                             "imu_quat", "body_to_imu_quat", "vicon_quat",
                             "commands", "vicon_pos", "vicon_goal_pos",
                             "radio_on", "aligner", "ahrs", "arming", "motors_on", "vicon_on", "sw_elevdr", "sw_flap", "posctrl_on",
                             "pid", "motor_coef", "pos_pid", "pos_cmd_limits", "ailevon_params", "battery_voltage" };
int number_of_fields = 31;
const int fieldnum_tick = 0;
const int fieldnum_mag  = 1;
const int fieldnum_gyro = 2;
const int fieldnum_acc  = 3;
const int fieldnum_actuators = 4; // 5
const int fieldnum_body_quat = 5;
const int fieldnum_goal_quat  = 6;
const int fieldnum_status = 7;

const int fieldnum_radio = 8;
const int fieldnum_rates = 9; // 10
const int fieldnum_imu_quat = 10;
const int fieldnum_body_to_imu_quat = 11;
const int fieldnum_vicon_quat = 12;
const int fieldnum_commands = 13;
const int fieldnum_vicon_pos = 14; // 15
const int fieldnum_vicon_goal_pos = 15;

const int fieldnum_status_radio = 16;
const int fieldnum_status_aligner = 17;
const int fieldnum_status_ahrs = 18;
const int fieldnum_status_arming = 19; // 20
const int fieldnum_status_motors = 20;
const int fieldnum_status_vicon = 21;
const int fieldnum_status_sw_elevdr = 22;
const int fieldnum_status_sw_flap = 23;
const int fieldnum_status_posctrl = 24; // 25

const int fieldnum_pid = 25;
const int fieldnum_motor_coef = 26;
const int fieldnum_pos_pid = 27;
const int fieldnum_pos_cmd_limits = 28;
const int fieldnum_ailevon_params = 29; // 30

const int fieldnum_battery_voltage = 30;


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
	double *x;
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

        if (openSerial(str_matlab, baud) == 0) {
            crc32_init();
            
            ClearSerialPortBuffers();

            kill_tread = 0;

            pthread_t thread;
            int recv_port = 1;
            pthread_create(&thread, NULL, receiver, (void *) &recv_port);
        }        
        return;
    } else if (strcmp(commandName, "test") == 0)
    {
        plhs[0] = mxCreateStructMatrix(1, 1, number_of_fields, fieldnames);
        
        mxSetFieldByNumber(plhs[0], 0, 0, mxCreateString(fieldnames[0]));
        mxSetFieldByNumber(plhs[0], 0, 1, mxCreateDoubleScalar(15));
        mxArray *val = mxCreateDoubleMatrix(2, 2, mxREAL);
        double *y = mxGetPr(val);
        y[0] = 1.0;
        y[1] = 2.0;
        y[2] = 3.0;
        y[3] = 4.0;
        mxSetFieldByNumber(plhs[0], 0, 2, val);
        
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
    } else if (strcmp(commandName, "send_pid") == 0)
    {
        if ((nrhs < 2) || (mxGetN(prhs[1]) == 4) || (mxGetM(prhs[1]) == 3)) {
            mexErrMsgTxt("Please pass a 4x3 matrix of PIDD values for pitch, roll, and yaw.");
        }
        x = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(1,4, mxREAL);
		z = mxGetPr(plhs[0]);
        writeSetupData(x, 12, 1, z);
    } else if (strcmp(commandName, "send_pos_pid") == 0)
    {
        if ((nrhs < 2) || (mxGetN(prhs[1]) == 4) || (mxGetM(prhs[1]) == 3)) {
            mexErrMsgTxt("Please pass a 4x3 matrix of position PIDD values for pitch, roll, and yaw.");
        }
        x = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(1,4, mxREAL);
		z = mxGetPr(plhs[0]);
        writeSetupData(x, 12, 3, z);
    } else if (strcmp(commandName, "send_mcoef") == 0)
    {
        if ((nrhs < 2) || (mxGetN(prhs[1]) == 4) || (mxGetM(prhs[1]) == 4)) {
            mexErrMsgTxt("Please pass a 4x4 matrix of mcoef values.");
        }
        x = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(1,4, mxREAL);
		z = mxGetPr(plhs[0]);
        writeSetupData(x, 16, 2, z);
    } else if (strcmp(commandName, "send_pos_cmd_limits") == 0)
    {
        if ((nrhs < 2) || (mxGetN(prhs[1]) == 4) || (mxGetM(prhs[1]) == 1)) {
            mexErrMsgTxt("Please pass a 4x1 matrix of pos cmd limit values.");
        }
        x = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(1,4, mxREAL);
		z = mxGetPr(plhs[0]);
        writeSetupData(x, 4, 4, z);
    } else if (strcmp(commandName, "send_ailevon_params") == 0)
    {
        if ((nrhs < 2) || (mxGetN(prhs[1]) == 4) || (mxGetM(prhs[1]) == 1)) {
            mexErrMsgTxt("Please pass a 4x1 matrix of ailevon parameters.");
        }
        x = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(1,4, mxREAL);
		z = mxGetPr(plhs[0]);
        writeSetupData(x, 4, 5, z);
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
    } else if (strcmp(commandName, "read_one") == 0)
    {
        sem_wait(&mutex);
        int ndim = count;
        sem_post(&mutex);
        if (ndim > 0) {
            readSerialIMU(plhs,1);
        } else {
            readSerialIMU(plhs,0);
        }
    } else if (strcmp(commandName, "read_all") == 0)
    {
        sem_wait(&mutex);
        int ndim = count;
        sem_post(&mutex);
		readSerialIMU(plhs, ndim);
    } else if (strcmp(commandName, "read_new") == 0)
    {
        sem_wait(&mutex);
        int ndim = count;
        if (count > 1) {
            ptr_out = (ptr_out + count - 1) % PKT_COUNT;
            count = 1;
        }
        sem_post(&mutex);
        plhs[1] = mxCreateDoubleScalar(ndim);
        if (ndim > 0) {
            readSerialIMU(plhs,1);
        } else {
            readSerialIMU(plhs,0);
        }
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

int readSerialIMU(mxArray *plhs[], int ndim) {
	int i, val, idim;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot read. Open serial port first");
    
    plhs[0] = mxCreateStructMatrix(1, ndim, number_of_fields, fieldnames);    
    
    sem_wait(&mutex);
    if (count >= ndim) {

        for (idim = 0; idim < ndim; idim++) {

            double tick = *((int*)&buff[ptr_out][0]); //answer[3] * 16777216 + answer[2] * 65536 + answer[1] * 256 + answer[0];
            mxSetFieldByNumber(plhs[0], idim, fieldnum_tick, mxCreateDoubleScalar(tick / 1000));
            
            //mxSetFieldByNumber(plhs[0], 0, 0, mxCreateString(fieldnames[0]));
            
            mxArray *mxMag = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dMag = mxGetPr(mxMag);
            mxArray *mxGyro = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dGyro = mxGetPr(mxGyro);
            mxArray *mxAcc = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dAcc = mxGetPr(mxAcc);
            for (i = 0; i < 3; i++) {
                val = buff[ptr_out][2*i+5] * 256 + buff[ptr_out][2*i+4];
                if (val > 32767) val = val - 65536;
                dMag[i] = (double)val;

                val = buff[ptr_out][2*i+11] * 256 + buff[ptr_out][2*i+10];
                if (val > 32767) val = val - 65536;
                dGyro[i] = (double)val;

                val = buff[ptr_out][2*i+17] * 256 + buff[ptr_out][2*i+16];
                if (val > 32767) val = val - 65536;
                dAcc[i] = (double)val;
            }            
            mxSetFieldByNumber(plhs[0], idim, fieldnum_mag, mxMag);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_gyro, mxGyro);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_acc, mxAcc);

            mxArray *mxActuators = mxCreateDoubleMatrix(1, 6, mxREAL);
            double *dActuators = mxGetPr(mxActuators);
            mxArray *mxRadio = mxCreateDoubleMatrix(1, 6, mxREAL);
            double *dRadio = mxGetPr(mxRadio);
            for (i = 0; i < 6; i++) {
                val = buff[ptr_out][2*i+23] * 256 + buff[ptr_out][2*i+22];
                if (val > 32767) val = val - 65536;
                dActuators[i] = (double)val;
                val = buff[ptr_out][2*i+35] * 256 + buff[ptr_out][2*i+34];
                if (val > 32767) val = val - 65536;
                dRadio[i] = (double)val;
            }            
            mxSetFieldByNumber(plhs[0], idim, fieldnum_actuators, mxActuators);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_radio, mxRadio);

            mxArray *mxRates = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dRates = mxGetPr(mxRates);
            mxArray *mxViconPos = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dViconPos = mxGetPr(mxViconPos);
            mxArray *mxViconGoalPos = mxCreateDoubleMatrix(1, 3, mxREAL);
            double *dViconGoalPos = mxGetPr(mxViconGoalPos);
            for (i = 0; i < 3; i++) {
                dRates[i]        = *((int*)&buff[ptr_out][4*i+ 46]) / 4096.0;
                dViconPos[i]     = *((int*)&buff[ptr_out][4*i+186]) / 1000.0;
                dViconGoalPos[i] = *((int*)&buff[ptr_out][4*i+198]) / 1000.0;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_rates, mxRates);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_vicon_pos, mxViconPos);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_vicon_goal_pos, mxViconGoalPos);
            
            mxArray *mxCommands = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dCommands = mxGetPr(mxCommands);
            mxArray *mxBodyQuat = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dBodyQuat = mxGetPr(mxBodyQuat);
            mxArray *mxGoalQuat = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dGoalQuat = mxGetPr(mxGoalQuat);
            mxArray *mxImuQuat = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dImuQuat = mxGetPr(mxImuQuat);
            mxArray *mxBodyToImuQuat = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dBodyToImuQuat = mxGetPr(mxBodyToImuQuat);
            mxArray *mxViconQuat = mxCreateDoubleMatrix(1, 4, mxREAL);
            double *dViconQuat = mxGetPr(mxViconQuat);
            for (i = 0; i < 4; i++) { // 3 (rates) + 8*4 (quats)
                dCommands[i]      = *((int*)&buff[ptr_out][4*i+ 58]);
                dBodyQuat[i]      = *((int*)&buff[ptr_out][4*i+ 74]) / 32768.0; //46
                dGoalQuat[i]      = *((int*)&buff[ptr_out][4*i+ 90]) / 32768.0;
                dImuQuat[i]       = *((int*)&buff[ptr_out][4*i+106]) / 32768.0;
                dBodyToImuQuat[i] = *((int*)&buff[ptr_out][4*i+122]) / 32768.0;
                dViconQuat[i]     = *((int*)&buff[ptr_out][4*i+138]) / 32768.0;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_commands, mxCommands);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_body_quat, mxBodyQuat);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_goal_quat, mxGoalQuat);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_imu_quat, mxImuQuat);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_body_to_imu_quat, mxBodyToImuQuat);
            mxSetFieldByNumber(plhs[0], idim, fieldnum_vicon_quat, mxViconQuat);

            mxArray *mxPID = mxCreateDoubleMatrix(4, 3, mxREAL);
            double *dPID = mxGetPr(mxPID);
            for (i = 0; i < 12; i++) {
                val = buff[ptr_out][2*i+211] * 256 + buff[ptr_out][2*i+210];
                if (val > 32767) val = val - 65536;
                dPID[i] = (double)val;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_pid, mxPID);
            
            mxArray *mxMotorCoef = mxCreateDoubleMatrix(4, 4, mxREAL);
            double *dMotorCoef = mxGetPr(mxMotorCoef);
            for (i = 0; i < 16; i++) {
                val = buff[ptr_out][2*i+235] * 256 + buff[ptr_out][2*i+234];
                if (val > 32767) val = val - 65536;
                dMotorCoef[i] = (double)val;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_motor_coef, mxMotorCoef);

            mxArray *mxPosPID = mxCreateDoubleMatrix(4, 3, mxREAL);
            double *dPosPID = mxGetPr(mxPosPID);
            for (i = 0; i < 12; i++) {
                val = buff[ptr_out][2*i+267] * 256 + buff[ptr_out][2*i+266];
                if (val > 32767) val = val - 65536;
                dPosPID[i] = (double)val;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_pos_pid, mxPosPID);

            mxArray *mxPosCmdLimits = mxCreateDoubleMatrix(4, 1, mxREAL);
            double *dPosCmdLimits = mxGetPr(mxPosCmdLimits);
            for (i = 0; i < 4; i++) {
                val = buff[ptr_out][2*i+291] * 256 + buff[ptr_out][2*i+290];
                if (val > 32767) val = val - 65536;
                dPosCmdLimits[i] = (double)val;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_pos_cmd_limits, mxPosCmdLimits);

            mxArray *mxAilevonParams = mxCreateDoubleMatrix(4, 1, mxREAL);
            double *dAilevonParams = mxGetPr(mxAilevonParams);
            for (i = 0; i < 4; i++) {
                val = buff[ptr_out][2*i+299] * 256 + buff[ptr_out][2*i+298];
                if (val > 32767) val = val - 65536;
                dAilevonParams[i] = (double)val;
            }
            mxSetFieldByNumber(plhs[0], idim, fieldnum_ailevon_params, mxAilevonParams);

            int battery_voltage_i = buff[ptr_out][307] * 256 + buff[ptr_out][306];
            double battery_voltage = (double)battery_voltage_i;
            if (battery_voltage > 32767) battery_voltage = battery_voltage - 65536;
            mxSetFieldByNumber(plhs[0], idim, fieldnum_battery_voltage, mxCreateDoubleScalar(battery_voltage / 1000));
            
            int statval = buff[ptr_out][PKT_SIZE-5] * 256 + buff[ptr_out][PKT_SIZE-6];

            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_radio, mxCreateDoubleScalar(statval & 0x03));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_aligner, mxCreateDoubleScalar((statval >>  2) & 0x03));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_ahrs, mxCreateDoubleScalar((statval >>  4) & 0x01));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_arming, mxCreateDoubleScalar((statval >>  5) & 0x07));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_motors, mxCreateDoubleScalar((statval >>  8) & 0x01));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_posctrl, mxCreateDoubleScalar((statval >>  9) & 0x01));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_sw_elevdr, mxCreateDoubleScalar((statval >> 10) & 0x01));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_sw_flap, mxCreateDoubleScalar((statval >> 11) & 0x01));
            mxSetFieldByNumber(plhs[0], idim, fieldnum_status_vicon, mxCreateDoubleScalar((statval >> 12) & 0x0f));
            
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

int writeSetupData(double *data, int size, int msg_id, double *dcrc) {
	char dout[40];
	int i, val;

    if (fdCom == 0)
        mexErrMsgTxt("Cannot write. Open serial port first");

    dout[0] = 0xff;
    dout[1] = 0xff;
    dout[2] = msg_id;
    dout[3] = 0;
    
	for (i = 0; i < size; i++) {
        if (data[i] > 32767) data[i] = 32767;
        if (data[i] < -32768) data[i] = -32768;
		val = (int)data[i];
		dout[2*i+4] = val & 0xff; 
		dout[2*i+5] = (val>>8) & 0xff; 
	}

    unsigned int crc32 = ~crc32_compute(0xffffffff, &dout[2], 34);
    *((int*)&dout[2+34]) = crc32;

    crc32 = ~crc32_compute(0xffffffff, &dout[2], 38);

    dcrc[0] = (double)((crc32) & 0xff);
    dcrc[1] = (double)((crc32 >> 8) & 0xff);
    dcrc[2] = (double)((crc32 >> 16) & 0xff);
    dcrc[3] = (double)((crc32 >> 24) & 0xff);
    
    SendComPortBlock(dout, 40);
    return 40;
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

