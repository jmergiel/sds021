#include <cstdio>
#include <cstdarg>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <algorithm>

#ifdef INTERACTIVE
#include <thread>
#endif

#include "scoped_exit.h"


static constexpr int CMD_BYTES = 19;
static constexpr int RESULT_BYTES = 10;
static constexpr int CMDID_MEAS = 0xC0;
static constexpr int CMDID_REPLY = 0xC5;

namespace
{
	void error_message(const char* msg, ...)
	{
		va_list va;
		va_start(va, msg);
		char buff[512]{};
		sprintf(buff, "ERROR [%s]\n", msg);
		vfprintf(stderr, buff, va);
		va_end(va);
	}

	void dump(const uint8_t* pBuff, int num)
	{
		for(int i=0; i<num; ++i) printf("%02x ", pBuff[i]);
	}

	std::string MakeTimestamp()
	{
		char buff[32]{};
		const time_t now{ time(nullptr) };
		strftime(buff, sizeof(buff), "%F.%T", gmtime(&now));
		return std::string(buff);
	}
}

namespace Cmd
{
	static constexpr int Head = 0;
	static constexpr int CmdID = 1;	// must be == CMDID_REPLY
	static constexpr int Type = 2;
	static constexpr int Data1 = Type;
	static constexpr int Data2 = 3;
	static constexpr int Data3 = 4;
	static constexpr int Data4 = 5;
	static constexpr int ID1 = 6;
	static constexpr int ID2 = 7;
	static constexpr int Checksum = 8;
	static constexpr int Tail = 9;
}

namespace Options
{
	// I need it simple and fast so found this on StackOverflow.com and altered a bit
	// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
	class InputParser
	{
		std::vector <std::string> tokens;
		public:
			InputParser(int argc, const char** argv)
			{
				for(int i=1; i < argc; ++i) tokens.emplace_back(argv[i]);
			}

			bool cmdOptionExists(const std::string& option) const
			{
				return std::find(tokens.begin(), tokens.end(), option) != tokens.end();
			}
	};

	static bool bInteractiveMode = false;
	static bool bJustNumbers = false;
}

std::string parseReply(const uint8_t (&V)[RESULT_BYTES])
{
	if(V[Cmd::CmdID] != CMDID_REPLY) return error_message("not a command reply"), "";
	
	char buff[512]{};
	
	switch(V[Cmd::Type])
	{
	case 6:	sprintf(buff, "mode %s: %s", V[Cmd::Data2] ? "set to" : "is", V[Cmd::Data3] ? "work" : "sleep"); break;
	case 8:	sprintf(buff, "work period %s: %d %s", V[Cmd::Data2] ? "set to" : "is", V[Cmd::Data3], V[Cmd::Data3] ? "minute(s)":"(continous)"); break;
	default: sprintf(buff, "unknown command reply type [%02x]", V[Cmd::Type]);
	};
	
	return buff;
}

void parse(const uint8_t (&V)[RESULT_BYTES], bool bJustNumbers)
{
	if(V[Cmd::Head] != 0xAA) error_message("header mismatch");
	// 0xC0: result of measurement
	// 0xC5: command reply
	if(V[Cmd::CmdID] != CMDID_MEAS && V[Cmd::CmdID] != CMDID_REPLY) error_message("command prefix mismatch");
	if(V[Cmd::Tail] != 0xAB) error_message("tail mismatch");
	{
		int check=0;
		for(int i=2; i<=7; ++i) check += V[i];
		if(V[Cmd::Checksum] != (check&0xFF)) error_message("checksum mismatch");
	}
		
	if(V[Cmd::CmdID] == CMDID_MEAS)
	{
		const float PM2_5 = (V[3] * 256 + V[2])/10.f;
		const float PM10 = (V[5] * 256 + V[4])/10.f;
		if(bJustNumbers) printf("%.01f %.01f\n", PM2_5, PM10);
		else printf("-> MEAS PM25=%05.01f PM10=%05.01f ID=%02X%02X\n", PM2_5, PM10, V[Cmd::ID1], V[Cmd::ID2]);
	}
	else
	{
		if(!bJustNumbers) printf("-> RPLY d1=%02x, d2=%02x, d3=%02x, d4=%02x ID=%02X%02X <%s>\n", V[2], V[3], V[4], V[5], V[Cmd::ID1], V[Cmd::ID2], parseReply(V).c_str());
	}
}

namespace Request
{
	constexpr uint8_t GetPeriod[] =		{ 0xAA, 0xB4, 0x08, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x15, 0xAB };
	constexpr uint8_t SetPeriodCont[] =	{ 0xAA, 0xB4, 0x08, 0x01, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x16, 0xAB };	// 0 == continuous mode
	constexpr uint8_t SetPeriod1m[] =	{ 0xAA, 0xB4, 0x08, 0x01, 0x01, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x17, 0xAB };	// [4] <- n (1-30) minutes
	constexpr uint8_t SetPeriod5m[] =	{ 0xAA, 0xB4, 0x08, 0x01, 0x05, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x1B, 0xAB };	// 5 minutes
	
	constexpr uint8_t GetMode[] =		{ 0xAA, 0xB4, 0x06, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x13, 0xAB };	// work
	constexpr uint8_t SetModeSleep[] =	{ 0xAA, 0xB4, 0x06, 0x01, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x14, 0xAB };	// sleep
	constexpr uint8_t SetModeWork[] =	{ 0xAA, 0xB4, 0x06, 0x01, 0x01, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x15, 0xAB };	// work
}

int set_interface_attribs(int fd,int speed,int parity)
{
	struct termios tty;
	memset(&tty,0,sizeof tty);
	if(tcgetattr(fd,&tty) != 0)
	{
		error_message("error %d from tcgetattr",errno);
		return -1;
	}

	cfsetospeed(&tty,speed);
	cfsetispeed(&tty,speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = RESULT_BYTES;  // read doesn't block
	tty.c_cc[VTIME] = 0;            // 0 seconds read timeout (no timeout)

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd,TCSANOW,&tty) != 0)
	{
		error_message("tcsetattr failed (errno=%d)",errno);
		return -1;
	}
	return 0;
}

bool SendCmd(const uint8_t (&V)[CMD_BYTES], int fd)
{
	int check=0;
	for(int i=2; i<=16; ++i) check += V[i];
	check &= 0xFF;
	if(check != V[17]) error_message("mismatched checksum in command (%02x)", check);
	else
	{
		printf("Sending command: ");
		dump(V, sizeof(V));
		printf("\n");
		if(write(fd, V, sizeof(V)) != sizeof(V))
		{
			error_message("write failed (errno=%d)", errno);
			return false;
		}
	}
	return true;
}

void ReadThread(int fd, int pip = -1)
{
	for(;;)
	{
		// this entire pip cruft is only used in interactive mode
		// to stop this thread in a correct, on-demand fashion
		if(pip != -1)
		{
			fd_set set;
			FD_ZERO(&set);
			FD_SET(fd, &set);
			FD_SET(pip, &set);
			
			if(select(FD_SETSIZE, &set, nullptr, nullptr, nullptr) == -1)
			{
				error_message("error from select\n");
				return;
			}
			if(FD_ISSET(pip, &set))
			{
				// we got something through the pipe! we care not
				// what it is -> this is a signal we should return
				return;
			}
			// if we're here it means there's data to be read from
			// the serial port - we just "fall-through"
		}
		
		uint8_t buf[RESULT_BYTES]{};
		const int nBytes = read(fd, buf, sizeof(buf));
		if(nBytes == RESULT_BYTES)
		{
			if(Options::bJustNumbers) parse(buf, true);
			else
			{
				printf("{%s} ", MakeTimestamp().c_str());
				dump(buf, sizeof(buf));
				parse(buf, false);
			}
			fflush(stdout);	//< force output now
		}
		else
		{
			error_message("read failed with result=%d (errno=%d)", nBytes, errno);
			return;
		}
	}
}

#ifdef INTERACTIVE
void Interactive(int fd)
{
	bool bOK = true;
	while(bOK)
	{
		switch(getchar())
		{
		case 'q':
		case 'Q': return;
		case 's': bOK = SendCmd(Request::SetModeSleep, fd); break;
		case 'w': bOK = SendCmd(Request::SetModeWork, fd); break;
		case 'p': bOK = SendCmd(Request::GetPeriod, fd); break;
		case 'c': bOK = SendCmd(Request::SetPeriodCont, fd); break;
		case '1': bOK = SendCmd(Request::SetPeriod1m, fd); break;
		case '5': bOK = SendCmd(Request::SetPeriod5m, fd); break;
		};
	}
}
#endif

int main(int argc, const char** argv)
{
	if(argc < 2)
	{
		printf("\nUSAGE:\t\t%s SERIAL_PORT_PATH [OPTIONS]\n", argv[0]);
		printf("EXAMPLE:\t%s /dev/ttyUSB0\n\n", argv[0]);
		printf("OPTIONS:\n");
		printf("\t-i\tInteractive mode on (default is off)\n");
		printf("\t-n\tOutput only (space separated) values of PM2.5 first and PM10 later (default is full protocol dump)\n");
		printf("\n\n");
		return -2;
	}

	const char* portname = argv[1];
	Options::InputParser cmd(argc-1, argv+1);
	Options::bInteractiveMode = cmd.cmdOptionExists("-i");
	Options::bJustNumbers = cmd.cmdOptionExists("-n");

	// The entire serial port code is from StackOverflow.com
	// with some modifications. I couldn't be bothered to learn
	// using it on Linux so if there's something wrong do tell!
	const int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		const int err = errno;
		error_message("Cannot open '%s': %s (error=%d)", portname, strerror(err), err);
		return -1;
	}
	auto scopedUART = make_scoped([=]{ close(fd); });
	set_interface_attribs(fd, B9600, 0);  //< set speed to 9600, 8n1 (no parity)

	if(!Options::bInteractiveMode) ReadThread(fd);
	else
	{
#ifdef INTERACTIVE
		printf("INTERACTIVE MODE!\n");
		int pip[2]{};
		if(pipe(pip))
		{
			error_message("pipe error\n");
			return -1;
		}
		auto scopedPipe = make_scoped([&]{ close(pip[0]); close(pip[1]); });
		
		std::thread reader(ReadThread, fd, pip[0]);
		Interactive(fd);
		static_cast<void>(1+write(pip[1], "Q", 1));	//< the +1 is a funky way of suppressing the unused return value warning
		reader.join();
#else
		error_message("Not compiled with interactive mode enabled! Use 'make interactive' to do that.");
#endif
	}

	return 0;
}

