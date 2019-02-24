///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <mgui/LogTray.h>
#include <iostream>

using namespace std;

//namespace mgui{

	LogTray *LogTray::mSingleton = nullptr;

	//---------------------------------------------------------------------------------------------------------------------
	void LogTray::init(const string _appName) {
		if (!mSingleton){
			mSingleton = new LogTray(_appName);
		}else{
			cout << "Someone tried to reinitialize the Log Telemetry";
			cout.flush();
		}	
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogTray::close(){
		delete mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogTray * LogTray::get(){
		return mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogTray::message(const std::string & _msg, bool _useCout) {
		double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
		std::string logLine = to_string(timeSpan) + " " + _msg + "\n";
		mSecureGuard.lock();
		mLogFile << logLine;
		mLogFile.flush(); 
		mSecureGuard.unlock();
		if(_useCout){
			cout << logLine;
			cout.flush();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogTray::LogTray(const std::string _appName) {
		mLogFile.open(_appName + to_string(time(NULL))+".txt");
		mInitTime = chrono::high_resolution_clock::now();
		cout << "Initialized Log Tray";
		cout.flush();
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogTray::~LogTray() {
		mLogFile.close();
	}

//}