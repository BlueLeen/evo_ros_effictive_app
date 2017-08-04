#include "ulog.h"
#include <fstream>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <log4cxx/logger.h>
#include <log4cxx/helpers/pool.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/simplelayout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/dailyrollingfileappender.h>
#include <log4cxx/rollingfileappender.h>
#ifdef ANDROIDLOG
#include <android/log.h>
#endif
#include "common_msgs/common.h"
#include "common_msgs/msgdata.h"

#ifdef ANDROIDLOG
#define    ANDROID_LOG_TAG		   "RAppLog"
#endif
#define    LOG_CMD_S2P_LOGCATINFO  MAKECMD(NODE_LOG, CMD_TYPE_FEEDBACK, 0x2)

std::string getName(const std::string & filePath)
{
	std::string fullPath = filePath;
	std::string name;
	int j = 0;
	for(int i=(int)fullPath.size()-1; i>=0; --i)
	{
		if(fullPath[i] == '/' || fullPath[i] == '\\')
		{
			j++;
			if(3 == j)
				break;
		}
		name.insert(name.begin(), fullPath[i]);
	}
	return name;
}

std::string uNumber2Str(int number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

std::string uNumber2Str(long number)
{
	std::stringstream s;
	s << number;
	return s.str();
}

std::string fmtMsg(const char *fmt, va_list arg)
{
    char szBuffer[1024] = { 0 };
    vsnprintf(szBuffer, sizeof(szBuffer), fmt, arg);
    std::string strBuf(szBuffer);
    return strBuf;
}

ULogger::Type ULogger::type_ = ULogger::kTypeNoLog; // Default nothing
ULogger::Level ULogger::level_ = kInfo; // By default, we show all info msgs + upper level (Warning, Error)
UMutex ULogger::loggerMutex_;
const std::string ULogger::kDefaultLogFileName = "applog.txt";
//bool ULogger::buffered_ = false;
ULogger* ULogger::instance_ = 0;
std::string ULogger::logFileName_ = ULogger::kDefaultLogFileName;
//std::string ULogger::bufferedMsgs_;
bool ULogger::append_ = true;
ros::NodeHandle* ULogger::pNh_ = NULL;
const std::string ULogger::levelStr_[kFatal+1] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

class UConsoleLogger : public ULogger
{
public :
    virtual ~UConsoleLogger() {}

protected:
    /**
     * Only the Logger can create inherited
     * loggers according to the Abstract factory patterns.
     */
    friend class ULogger;

    UConsoleLogger() {}

private:
//    virtual void _write(const char* msg, va_list arg)
//    {
//		vprintf(msg, arg);
//    }
//    virtual void _writeStr(ULogger::Level level, const char* msg)
//	{
//    	printf("%s", msg);
//	}
//    virtual void _write(const char *fmt, ...)
//    {
//        char szBuffer[1024] = { 0 };
//        va_list ap;
//        va_start(ap, fmt);
//        vsnprintf(szBuffer, sizeof(szBuffer), fmt, ap);
//        va_end(ap);
//        char szText[1024] = { 0 };
//        snprintf(szText, sizeof(szText), "[F:%s L:%d]: %s", __FILE__, __LINE__, szBuffer);
//		switch(level_)
//		{
//		case kDebug:
//			ROS_DEBUG("%s", szText);
//			break;
//		case kInfo:
//			ROS_INFO("%s", szText);
//			break;
//		case kWarning:
//			ROS_WARN("%s", szText);
//			break;
//		case kError:
//			ROS_ERROR("%s", szText);
//			break;
//		case kFatal:
//			ROS_FATAL("%s", szText);
//			break;
//		default:
//			ROS_DEBUG("%s", szText);
//			break;
//		}
//    }
    virtual void _writeStr(const char* msg)
    {
		switch(level())
		{
#ifndef ANDROIDLOG
		case kDebug:
			ROS_DEBUG("%s", msg);
			break;
		case kInfo:
			ROS_INFO("%s", msg);
			break;
		case kWarning:
			ROS_WARN("%s", msg);
			break;
		case kError:
			ROS_ERROR("%s", msg);
			break;
		case kFatal:
			ROS_FATAL("%s", msg);
			break;
		default:
			ROS_DEBUG("%s", msg);
			break;
#else
		case kDebug:
			 __android_log_write(ANDROID_LOG_DEBUG, ANDROID_LOG_TAG, msg);
			break;
		case kInfo:
			__android_log_write(ANDROID_LOG_INFO, ANDROID_LOG_TAG, msg);
			break;
		case kWarning:
			__android_log_write(ANDROID_LOG_WARN, ANDROID_LOG_TAG, msg);
			break;
		case kError:
			__android_log_write(ANDROID_LOG_ERROR, ANDROID_LOG_TAG, msg);
			break;
		case kFatal:
			__android_log_write(ANDROID_LOG_FATAL, ANDROID_LOG_TAG, msg);
			break;
		default:
			__android_log_write(ANDROID_LOG_DEBUG, ANDROID_LOG_TAG, msg);
			break;
#endif
		}
    }
};

//class UFileLogger : public ULogger
//{
//public:
//    virtual ~UFileLogger()
//    {
////    	this->_flush();
//    	if(fout_)
//    	{
//    		fclose(fout_);
//    	}
//    }
//
//protected:
//    /**
//     * Only the Logger can create inherited
//     * loggers according to the Abstract factory patterns.
//     */
//    friend class ULogger;
//
//    UFileLogger(const std::string &fileName, bool append)
//    {
//        fileName_ = fileName;
//        if(!append) {
//			std::ofstream fileToClear(fileName_.c_str(), std::ios::out);
//			fileToClear.clear();
//			fileToClear.close();
//		}
//        fout_ = fopen(fileName_.c_str(), "a");
//        if(!fout_) {
//            printf("FileLogger : Cannot open file : %s\n", fileName_.c_str()); // TODO send Event instead, or return error code
//            return;
//        }
//    }
//
//private:
//    virtual void _write(const char* msg, va_list arg)
//    {
//    	if(fout_)
//    	{
//    		vfprintf(fout_, msg, arg);
//    	}
//    }
//    virtual void _writeStr(const char* msg)
//	{
//		if(fout_)
//		{
//			fprintf(fout_, "%s", msg);
//		}
//	}
//private:
//    std::string fileName_; ///< the file name
//    FILE* fout_;
//    std::string bufferedMsgs_;
//};

class UFileLogger : public ULogger
{
public:
    virtual ~UFileLogger()
    {
    }

protected:
    /**
     * Only the Logger can create inherited
     * loggers according to the Abstract factory patterns.
     */
    friend class ULogger;

    UFileLogger(const std::string &fileName, bool append)
    {
    	std::ostringstream oss;
    	oss << getpid();
//    	std::string pid= oss.str();
//    	log4cxx::MDC::put("pid", oss.str());
    	log4cxx::PatternLayoutPtr layout = new log4cxx::PatternLayout();
//    	std::string conversionPattern = "[%p] %d %c %M - %m%n";
//    	conversionPattern = layout->getConversionPattern();
    	std::string conversionPattern = "[(%d) %p] %m%n";
    	layout->setConversionPattern(conversionPattern);
    	//log4cxx::DailyRollingFileAppenderPtr rollingAppenderptr = new log4cxx::DailyRollingFileAppender();
    	log4cxx::RollingFileAppenderPtr rollingAppenderptr = new log4cxx::RollingFileAppender();
//    	std::string fileName
    	fileName_ = "/app/log/"+ fileName /*+ "-" + pid*/;
    	rollingAppenderptr->setFile(fileName_);
    	rollingAppenderptr->setAppend(append);
    	rollingAppenderptr->setMaxFileSize("4MB");
    	rollingAppenderptr->setMaxBackupIndex(12);
//    	int size = rollingAppenderptr->getBufferSize();
//    	printf("%d", size);
    	//rollingAppenderptr->setFile("/home/su/Workspace/robot/ros_app_my/uLog."+pid, true, true, );
    	//rollingAppenderptr->setDatePattern("'.'yyyy-MM-dd");
    	rollingAppenderptr->setLayout(layout);
    	log4cxx::helpers::Pool p;
    	rollingAppenderptr->activateOptions(p);
    	log4cxx::BasicConfigurator::configure(rollingAppenderptr);
    	logger_ = log4cxx::Logger::getRootLogger();
    	logger_->setLevel(log4cxx::Level::getDebug());
    	logger_->addAppender(rollingAppenderptr);
    	//LOG4CXX_INFO(logger_,"Created FileAppender appender");
    }

private:
//    virtual void _write(const char* msg, va_list arg)
//    {
//    }
    virtual void _writeStr(const char* msg)
	{
		switch(level())
		{
		case kDebug:
			LOG4CXX_DEBUG(logger_, msg);
			break;
		case kInfo:
			LOG4CXX_INFO(logger_, msg);
			break;
		case kWarning:
			LOG4CXX_WARN(logger_, msg);
			break;
		case kError:
			LOG4CXX_ERROR(logger_, msg);
			break;
		case kFatal:
			LOG4CXX_FATAL(logger_, msg);
			break;
		default:
			LOG4CXX_DEBUG(logger_, msg);
			break;
		}
	}
private:
    std::string fileName_; ///< the file name
    //std::string bufferedMsgs_;
    log4cxx::LoggerPtr logger_;
};

class UCatLogger : public ULogger
{
    virtual ~UCatLogger()
    {
    }

protected:
    friend class ULogger;
	UCatLogger()
	{
		if(pNh_ != NULL)
			pubLog_ = pNh_->advertise<common_msgs::msgdata>("serial_cmd_to_pad", 10);
	}
private:
	virtual void _writeStr(const char* msg)
	{
		if(pubLog_)
		{
		    common_msgs::msgdata tmp_msg;
		    tmp_msg.cmd = LOG_CMD_S2P_LOGCATINFO;
		    tmp_msg.inlen = strlen(msg) + 1;
		    tmp_msg.input.resize(tmp_msg.inlen);
		    memcpy(&tmp_msg.input[0], msg, tmp_msg.inlen);
		    pubLog_.publish(tmp_msg);
		}
	}
private:
	ros::Publisher pubLog_;
};

ULogger::~ULogger()
{
    instance_ = 0;
}

void ULogger::setType(Type type, ros::NodeHandle* pNh, const std::string &fileName, bool append)
{
	//ULogger::flush();
    loggerMutex_.lock();
    {
		// instance not yet created
		if(!instance_)
		{
			type_ = type;
			pNh_ = pNh;
			logFileName_ = fileName;
			append_ = append;
			instance_ = createInstance();
		}
		// type changed
		else if(type_ != type || (type_ == kTypeFile && logFileName_.compare(fileName)!=0))
		{
			//destroyer_.setDoomed(0);
			delete instance_;
			instance_ = 0;
			type_ = type;
			pNh_ = pNh;
			logFileName_ = fileName;
			append_ = append;
			instance_ = createInstance();
		}
    }
    loggerMutex_.unlock();
}

//void ULogger::setType(Type type, ros::NodeHandle* pNh)
//{
//	loggerMutex_.lock();
//	{
//		if(!instance_)
//		{
//			type_ = type;
//			pNh_ = pNh;
//			instance_ = createInstance();
//		}
//		else if(type_ != type)
//		{
//			delete instance_;
//			instance_ = 0;
//			type_ = type;
//			pNh_ = pNh;
//			instance_ = createInstance();
//		}
//	}
//	loggerMutex_.unlock();
//}

//void ULogger::_flush()
//{
//	ULogger::getInstance()->_writeStr(bufferedMsgs_.c_str());
//	bufferedMsgs_.clear();
//}

ULogger* ULogger::getInstance()
{
	if(!instance_)
	{
		instance_ = createInstance();
	}
    return instance_;
}

ULogger* ULogger::createInstance()
{
    ULogger* instance = 0;
    if(type_ == ULogger::kTypeConsole)
    {
        instance = new UConsoleLogger();
    }
    else if(type_ == ULogger::kTypeFile)
    {
        instance = new UFileLogger(logFileName_, append_);
    }
    else if(type_ == ULogger::kTypeLogcat)
    {
    	instance = new UCatLogger();
    }
    return instance;
}

void ULogger::write(ULogger::Level level,
		const char * file,
		int line,
		const char * function,
		const char* format,
		...)
{
	loggerMutex_.lock();
	if(type_ == kTypeNoLog && level < kFatal)
	{
		loggerMutex_.unlock();
		return;
	}
	if(strlen(format) == 0 && level < kFatal)
	{
		loggerMutex_.unlock();
		// No need to show an empty message if we don't print where.
		return;
	}
	if(level >= level_)
	{
		std::string fullMsg = "";
//		fullMsg.append("[F:");
//		std::string fileName = getName(file);
//		fullMsg.append(fileName);
//		fullMsg.append(" ");
//		fullMsg.append("L:");
//		std::string strLine = uNumber2Str(line);
//		fullMsg.append(strLine);
//		fullMsg.append(" ");
//		fullMsg.append("func:");
//		fullMsg.append(function);
//		fullMsg.append(" ");
//		fullMsg.append("]: ");
		va_list args;
		va_start(args, format);
		std::string strMsg = fmtMsg(format, args);
		va_end(args);
		fullMsg.append(strMsg);
		if(type_ != kTypeConsole)
		{
			std::string LocMsg = "";
			if(type_ == kTypeLogcat)
			{
				LocMsg.append("[");
				LocMsg.append(levelStr_[level]);
				LocMsg.append("] ");
			}
//		    struct timeval tm;
//		    gettimeofday(&tm, NULL);
//		    std::string strTime = uNumber2Str(tm.tv_sec);
//		    LocMsg.append("[");
//		    LocMsg.append(strTime);
//		    strTime = uNumber2Str(tm.tv_usec);
//		    LocMsg.append(".");
//		    LocMsg.append(strTime);
//		    LocMsg.append("]: ");
//		    fullMsg.insert(0, LocMsg);
		}
		ULogger::getInstance()->_writeStr(fullMsg.c_str());
	}
	loggerMutex_.unlock();
}
