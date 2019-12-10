/*
 * CLog.h
 *
 *  Created on: 2019/12/06
 *      Author: KENSUKE
 */

#ifndef LOG_CLOG_H_
#define LOG_CLOG_H_
#include <stdio.h>
#include <assert.h>

class CLog {


public:
	CLog();
	virtual ~CLog();

	static void Trace(const char* message);
	static void Debug(const char* message);
	static void Info(const char* message);
	static void Warn(const char* message);
	static void Error(const char* message);
	static void Fatal(const char* message);

protected:
	static void WriteMessage(const char* level, const char* message) {
		assert(NULL != level);
		assert(NULL != message);

		printf("[%s] : %s\n", level, message);
	}

protected:
	static const char* INFO_TRACE_TAG;
	static const char* INFO_DEBUG_TAG;
	static const char* INFO_INFO_TAG;
	static const char* INFO_WARN_TAG;
	static const char* INFO_ERROR_TAG;
	static const char* INFO_FATAL_TAG;
};

#endif /* LOG_CLOG_H_ */
