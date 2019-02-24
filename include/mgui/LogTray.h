///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MGUI_LOGTRAY_H_
#define MGUI_LOGTRAY_H_

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>

//namespace mgui{

	/// Thread safe class used as logging system. 
	class LogTray {
	public:	//	Static interface.
		/// Initialize the logging system. 
		/// \param _appName: Base name used for the log file.
		/// \param _useCout: Write to cout too.
		static void init(const std::string _appName);

		/// Close the logging system. It makes sure that the log is closed properly.
		static void close();

		/// Get current instance of the logging system.
		static LogTray* get();

	public:	// Public interface.
		/// Write message to the log system
		void message(const std::string &_msg, bool _useCout = false);

	private:	// Private interface.
		LogTray(const std::string _appName);
		~LogTray();

		static LogTray *mSingleton;

		bool mUseCout = false;

		std::chrono::high_resolution_clock::time_point  mInitTime;
		std::ofstream mLogFile;
		std::mutex mSecureGuard;
	};
//}

#endif
