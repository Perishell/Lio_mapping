//
// Created by shulei.sheng on 2021/8/30.
//
#ifndef UGV_LOGMGMT_H
#define UGV_LOGMGMT_H
#include "spdlog/spdlog.h"
#include <set>
class LogMgmt
{
public:
    LogMgmt();
    ~LogMgmt();
    void InitLogger(std::string logger_name,std::string file_name, int log_level= spdlog::level::trace);
    void SetLevel(int level = spdlog::level::trace);
    std::shared_ptr<spdlog::logger> &GetLogger();
    static std::shared_ptr<LogMgmt>& GetInstance();
private:
    bool IsDirectory(const std::string &path);
    /**  以字节为单位返回文件的大小  **/
    bool GetFileSize(const std::string &file, unsigned long long &fileSize);
    /**  返回dir目录下所有文件的总大小    **/
    bool GetTotalFileSize(const std::string &dir, unsigned long long &totalFileSize);
    std::string AppendPath(const std::string &dir, const std::string &path);
    std::string GetPosixPath(const std::string &path);
    bool FileRemove(const std::string &file);
    bool GetFiles(const std::string &dir, std::set<std::string> &files);
    static std::shared_ptr<LogMgmt> m_ptrLogger;
    std::shared_ptr<spdlog::logger> m_ptrFileLogger;
    std::shared_ptr<spdlog::logger> m_ptrConsoler;
    std::thread m_thrUpdateThread;
};
#endif //UGV_LOGMGMT_H
