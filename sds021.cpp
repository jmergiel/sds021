#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <ctime>
#include <thread>
#include <chrono>
#include <csignal>
#include <functional>


static constexpr int CMD_BYTES = 19;
static constexpr int RESULT_BYTES = 10;
static constexpr int CMDID_MEAS = 0xC0;
static constexpr int CMDID_REPLY = 0xC5;

namespace
{
	class OnScopeExit final
	{
		std::function<void()>	m_func;
		bool					m_call = true;
	public:
		explicit OnScopeExit(std::function<void()> foo) : m_func(std::move(foo)) {}
		~OnScopeExit()
		{
			if(m_call && m_func) m_func();
		}
		void Dismiss() noexcept { m_call = false; }
	};

	template <typename T>
	OnScopeExit make_scoped(T&& foo)
	{
		return OnScopeExit(std::forward<T>(foo));
	}

	void error_message(const char* msg, ...)
	{
		va_list va;
		va_start(va, msg);
		char buff[512]{};
		sprintf(buff, "ERROR [%s]\n", msg);
		vfprintf(stdout, buff, va);
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

void parse(const uint8_t (&V)[RESULT_BYTES])
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
		printf("-> MEAS PM25=%05.01f PM10=%05.01f ID=%02X%02X\n", PM2_5, PM10, V[Cmd::ID1], V[Cmd::ID2]);
	}
	else
	{
		printf("-> RPLY d1=%02x, d2=%02x, d3=%02x, d4=%02x ID=%02X%02X <%s>\n", V[2], V[3], V[4], V[5], V[Cmd::ID1], V[Cmd::ID2], parseReply(V).c_str());
	}
}

namespace Request
{
	constexpr uint8_t GetPeriod[] =		{ 0xAA, 0xB4, 0x08, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x15, 0xAB };
	constexpr uint8_t SetPeriodCont[] =	{ 0xAA, 0xB4, 0x08, 0x01, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x16, 0xAB };	// 0 == continous mode
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
		error_message("error %d from tcsetattr",errno);
		return -1;
	}
	return 0;
}

void SendCmd(const uint8_t (&V)[CMD_BYTES], int fd)
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
		if(write(fd, V, sizeof(V)) == sizeof(V))
		{
			printf("Wrote command, waiting for reply...\n");
		}
	}
}

void ReadThread(int fd, int pip = -1)
{
	//SendCmd(Request::SetModeSleep, fd);
	for(;;)
	{
		uint8_t buf[RESULT_BYTES];

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
				//printf("pipe! so quit\n");
				return;
			}
			//if(FD_ISSET(fd, &set)) printf("COM data\n");
		}
		
		const int nBytes = read(fd, buf, sizeof(buf));
		//printf("nBytes=%d\n", nBytes);
		if(nBytes == RESULT_BYTES)
		{
			printf("%s: ", MakeTimestamp().c_str());
			dump(buf, sizeof(buf));
			parse(buf);
		}
		else if(nBytes < 0)
		{
			//printf("negative - bye\n");
			return;
		}
	}
}

#ifdef INTERACTIVE
void Interactive(int fd)
{
	for(;;)
	{
		switch(getchar())
		{
		case 'q':
		case 'Q': return;
		case 's': SendCmd(Request::SetModeSleep, fd); break;
		case 'w': SendCmd(Request::SetModeWork, fd); break;
		case 'p': SendCmd(Request::GetPeriod, fd); break;
		case 'c': SendCmd(Request::SetPeriodCont, fd); break;
		case '1': SendCmd(Request::SetPeriod1m, fd); break;
		case '5': SendCmd(Request::SetPeriod5m, fd); break;
		};
	}
}
#endif

int main()
{
	const char *portname = "/dev/ttyUSB0";
	const int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		error_message("error %d opening %s: %s", errno, portname, strerror(errno));
		return -1;
	}
	auto scopedUART = make_scoped([=]{ close(fd); });
	set_interface_attribs(fd, B9600, 0);  // set speed to 9600, 8n1 (no parity)

	bool bInteractive = true;
	if(!bInteractive) ReadThread(fd);
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
		static_cast<void>(1+write(pip[1], "Q", 1));	//< funky way of suppressing the unused return value warning
		reader.join();
#else
		error_message("Not compiled with interactive mode enabled! Use 'make interactive' to do that.");
#endif
	}

	printf("BYE BYE\n");
	return 0;
}

