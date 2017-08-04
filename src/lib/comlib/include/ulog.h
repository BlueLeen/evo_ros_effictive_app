#ifndef _ULOG_EVOLVER_H_
#define _ULOG_EVOLVER_H_

#include <iostream>
#include <ros/ros.h>
#include "umutex.h"

#define ULOGGER_LOG(level, ...) ULogger::write(level, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

#define ULOGGER_DEBUG(...)   ULOGGER_LOG(ULogger::kDebug,   __VA_ARGS__)
#define ULOGGER_INFO(...)    ULOGGER_LOG(ULogger::kInfo,    __VA_ARGS__)
#define ULOGGER_WARN(...) 	 ULOGGER_LOG(ULogger::kWarning, __VA_ARGS__)
#define ULOGGER_ERROR(...)   ULOGGER_LOG(ULogger::kError,   __VA_ARGS__)
#define ULOGGER_FATAL(...)   ULOGGER_LOG(ULogger::kFatal,   __VA_ARGS__) // Throw UException

#define UDEBUG(...)   ULOGGER_DEBUG(__VA_ARGS__)
#define UINFO(...)    ULOGGER_INFO(__VA_ARGS__)
#define UWARN(...) 	  ULOGGER_WARN(__VA_ARGS__)
#define UERROR(...)   ULOGGER_ERROR(__VA_ARGS__)
#define UFATAL(...)   ULOGGER_FATAL(__VA_ARGS__) // Throw UException

class ULogger
{
public:
    /**
     * The default log file name.
     */
    static const std::string kDefaultLogFileName;
    /**
     * Loggers available:
     * @code
     * kTypeNoLog, kTypeConsole, kTypeFile
     * @endcode
     */
    enum Type{kTypeNoLog=0x00, kTypeConsole=0x01,
    		  kTypeFile=0x02, kTypeLogcat=0x04,
    		  kConsoleFile=0x03, kConsoleLogcat=0x05,
    		  kFlieLogcat=0x06, kAll=0x07};
    /**
     * Logger levels, from lowest severity to highest:
     * @code
     * kDebug, kInfo, kWarning, kError, kFatal
     * @endcode
     */
    enum Level{kDebug, kInfo, kWarning, kError, kFatal};

    static void setType(Type type, ros::NodeHandle* pNh = NULL, const std::string &fileName = kDefaultLogFileName, bool append = true);
    static Type type() {return type_;}
    static void setLevel(ULogger::Level level) {level_ = level;}
    static ULogger::Level level() {return level_;}
    static void write(ULogger::Level level,
    		const char * file,
    		int line,
    		const char *function,
    		const char* format,
    		...);
    static ULogger* getInstance();

protected:
    ULogger() {}
    virtual ~ULogger();
//    void _flush();

    static std::string logFileName_;
    static bool append_;
    static ros::NodeHandle* pNh_;


private:
    static ULogger* createInstance();
    virtual void _writeStr(const char* msg) {} // Do nothing by default
    /*
     * The type of the logger.
     */
    static Type type_;

    /*
	 * The severity of the log.
	 */
    static Level level_;


    /*
     * Mutex used when accessing public functions.
     */
    static UMutex loggerMutex_;

    /*
     * The Logger instance pointer.
     */
    static ULogger* instance_;

    /*
	 * If the logger prints messages only when ULogger::flush() is called.
	 * Default is false.
	 */
	//static bool buffered_;

	//static std::string bufferedMsgs_;

	static const std::string levelStr_[kFatal+1];
};



#endif
