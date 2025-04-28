//
// Created by shulei.sheng on 2021/8/30.
//
#include "slog/LogMgmt.h"
#include "ros/ros.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <dirent.h>
std::shared_ptr<LogMgmt> LogMgmt::m_ptrLogger;

LogMgmt::LogMgmt()
{
    ros::NodeHandle nh("~");
    std::string path;
    int logLevel = 0;
    int logSize = 0;
    int logDays = 0;
    nh.param("log_level", logLevel, 2);
    nh.param("log_size", logSize, 20);//MB单位
    nh.param("log_days", logDays, 7);//天单位
    std::string subStr = "fusion_slam";//nh.getNamespace().substr(1, sizeof(nh.getNamespace()));
    char cmd[32] = "rospack find ";
    strcat(cmd, subStr.c_str());
    FILE *fp = popen(cmd, "r");
    char* packagePath = new char[256];
    fgets(packagePath, 256, fp);
    packagePath[strlen(packagePath) - 1] = 0;
    pclose(fp);
    std::string packagePathStr = packagePath;
    int ePos = packagePathStr.find("/src");
    std::string splitStr = packagePathStr.substr(0, ePos);
    int bPos = 0;
    for(int i = ePos; i >= 0; i--)
    {
        if(splitStr[i] == '/')
        {
            bPos = i;
            break;
        }
    }
    std::string workSpace = splitStr.substr(bPos + 1, ePos);
    path = getenv("HOME") + std::string("/logs/") + workSpace + nh.getNamespace();
    // path = "/disk3/workspace" + std::string("/logs/") + workSpace + nh.getNamespace();
//    判断文件夹是否存在
    if(access(path.c_str(), 0) == -1)
    {
//        创建多层文件夹
        printf("%s do not exit! \n", path.c_str());
        std::string order = "mkdir -p";
        order = order + std::string(" ") + path;
        auto res = system(order.c_str());
    }
    unsigned long long totalFileSize = 0;
    GetTotalFileSize(path, totalFileSize);
    if(totalFileSize > logSize * 1024 * 1024)
    {
        printf("log files total size too large, need remove old logs...\n");
        std::set<std::string> files;
        GetFiles(path, files);
        for(std::set<std::string>::iterator iter = files.begin(); iter != files.end(); iter++)
        {
            struct stat buf;
            if(0 == stat((*iter).c_str(), &buf))
            {
                time_t t;
                tzset();
                t = time(NULL);
                if((t - buf.st_mtime) > 86400 * logDays)
                {
                    FileRemove((*iter).c_str());
                }
            }
        }
    }
    path = path + nh.getNamespace() + std::string(".log");
    InitLogger(subStr, path,
               static_cast<spdlog::level::level_enum>(logLevel));
    m_thrUpdateThread = std::thread([this](){
        while(ros::ok())
        {
            this->GetLogger()->flush();
            //每x秒写入一次文件
            sleep(1);
        }
    });
    m_thrUpdateThread.detach();
}
LogMgmt::~LogMgmt()
{
//    关闭所有logger
    spdlog::drop_all();
}
std::shared_ptr<LogMgmt> &LogMgmt::GetInstance()
{
    if(m_ptrLogger == nullptr)
    {
        m_ptrLogger = std::make_shared<LogMgmt>();
    }
    return m_ptrLogger;
}
void LogMgmt::InitLogger(
        std::string logger_name, std::string file_name, int log_level)
{
    size_t q_size = 4096; // queue size must be power of 2MB
    spdlog::set_async_mode(q_size);
    spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level));
    m_ptrFileLogger = spdlog::daily_logger_mt(logger_name, file_name);
    m_ptrFileLogger->flush_on(spdlog::level::info);
    m_ptrFileLogger->info("Logger Start !");
    m_ptrConsoler = spdlog::stdout_color_mt("logger");
}
void LogMgmt::SetLevel(int level)
{
    spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
}
std::shared_ptr<spdlog::logger> &LogMgmt::GetLogger()
{
    return m_ptrFileLogger;
}
bool LogMgmt::IsDirectory(const std::string &path)
{
    struct stat buf;
    if (0 != stat(path.c_str(), &buf))
    {
        /**  path不存在，则返回ENOENT，其他失败原因，则打印一下  **/
        if (ENOENT != errno)
        {
            std::cout << "IsDirectory: stat fail, path=" << path << ", errno=" << errno << ", " << strerror(errno) << std::endl;
        }
        return false;
    }
    return S_ISDIR(buf.st_mode);
}
bool LogMgmt::GetFileSize(const std::string &file, unsigned long long &fileSize)
{
    struct stat buf;
    if (0 != stat(file.c_str(), &buf))
    {
        std::cout << "GetFileSize: stat fail, file=" << file << ", errno=" << errno << ", " << strerror(errno) << std::endl;
        return false;
    }
    fileSize = buf.st_size;
    return true;
}
bool LogMgmt::GetTotalFileSize(const std::string &dir, unsigned long long &totalFileSize)
{
    totalFileSize = 0;
    DIR* pDIR = opendir(dir.c_str());
    if (NULL == pDIR) return false;

    struct dirent *pDirent;
    for(; (pDirent = readdir(pDIR)) != NULL;)
    {
        std::string file = AppendPath(dir, pDirent->d_name);
        unsigned long long fileSize(0);
        if(!IsDirectory(file))
        {
            if (!GetFileSize(file, fileSize))
            {
                closedir(pDIR);
                return false;
            }
            totalFileSize += fileSize;
            continue;
        }
    }
    closedir(pDIR);
    return true;
}
std::string LogMgmt::AppendPath(const std::string &dir, const std::string &path)
{
    std::string filename(GetPosixPath(path));
    if (!filename.empty() && '/' == filename[0])
    {
        filename = 1 == filename.size() ? "" : filename.substr(1);
    }
    std::string posixDir(GetPosixPath(dir));
    if (posixDir.empty() || '/' != posixDir[posixDir.size()-1])
    {
        return posixDir + "/" + filename;
    }
    return posixDir + filename;
}
std::string LogMgmt::GetPosixPath(const std::string &path)
{
    std::string posixPath(path);
    for(unsigned int i = 0; i < posixPath.size(); i++)
    {
        if (posixPath[i] == '\\') posixPath[i] = '/';
    }
    return posixPath;
}
bool LogMgmt::FileRemove(const std::string &file)
{
    if (0 != remove(file.c_str()))
    {
        std::cout << "FileRemove: remove fail, file=" << file << ", errno=" << errno << ", " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}
bool LogMgmt::GetFiles(const std::string &dir, std::set<std::string> &files)
{
    if (!IsDirectory(dir)) return false;

    DIR* pDIR = opendir(dir.c_str());
    if (NULL == pDIR) return false;

    struct dirent *pDirent;
    for(; (pDirent = readdir(pDIR)) != NULL;)
    {
        std::string file = AppendPath(dir, pDirent->d_name);
        if (!IsDirectory(file))
        {
            files.insert(file);
            continue;
        }
    }
    closedir(pDIR);
    return true;
}