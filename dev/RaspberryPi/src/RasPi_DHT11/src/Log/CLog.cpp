/*
 * CLog.cpp
 *
 *  Created on: 2019/12/06
 *      Author: CountrySideEngineer.
 */

#include "CLog.h"

const char* CLog::INFO_TRACE_TAG = "TRACE";
const char* CLog::INFO_DEBUG_TAG = "DEBUG";
const char* CLog::INFO_INFO_TAG =  "INFO ";
const char* CLog::INFO_WARN_TAG =  "WARN ";
const char* CLog::INFO_ERROR_TAG = "ERROR";
const char* CLog::INFO_FATAL_TAG = "FATAL";

CLog::CLog() {}
CLog::~CLog() {}

void CLog::Trace(const char* message) { CLog::WriteMessage(INFO_TRACE_TAG, message); }
#ifdef __DEBUG
void CLog::Debug(const char* message) { CLog::WriteMessage(INFO_DEBUG_TAG, message); }
#else
void CLog::Debug(const char* message) { }
#endif
void CLog::Info(const char* message) { CLog::WriteMessage(INFO_INFO_TAG, message); }
void CLog::Warn(const char* message) { CLog::WriteMessage(INFO_WARN_TAG, message); }
void CLog::Error(const char* message) { CLog::WriteMessage(INFO_ERROR_TAG, message); }
void CLog::Fatal(const char* message) { CLog::WriteMessage(INFO_FATAL_TAG, message); }
