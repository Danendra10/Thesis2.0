#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace std;

#define RED "\033[0;31m"
#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define BLUE "\033[0;34m"
#define MAGENTA "\033[0;35m"
#define CYAN "\033[0;36m"
#define WHITE "\033[0;37m"
#define RESET "\033[0m"

string log_dir;

void SaveLog(const char *msg, ...);

int InitLogger()
{
    filesystem::path log_directory = filesystem::current_path() / "../log";
    filesystem::path log_path = log_directory / "logger.log";

    if (!filesystem::exists(log_directory))
    {
        filesystem::create_directories(log_directory);
        ofstream file_name(log_path);
    }

    log_dir = log_path.string();
}

void Logger(const char *color, const char *msg, ...)
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char buffer[100];
    sprintf(buffer, "[%d-%d-%d %d:%d:%d] => ", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

    // Create a dynamic format string with placeholders replaced by "%s"
    va_list args;
    va_start(args, msg);
    char dynamicFormat[200];
    vsprintf(dynamicFormat, msg, args);
    va_end(args);

    // Combine the timestamp, prefix, and dynamic message into a single message
    char combinedMsg[300];
    snprintf(combinedMsg, sizeof(combinedMsg), "%s%s", buffer, dynamicFormat);

    SaveLog(combinedMsg);

    va_start(args, msg);
    printf("%s\n", color);
    vprintf(combinedMsg, args);
    printf("%s", RESET);
    va_end(args);
}

void SaveLog(const char *msg, ...)
{
    char buffer[200];
    time_t now = time(0);
    tm *ltm = localtime(&now);

    va_list args;
    va_start(args, msg);

    vsnprintf(buffer, sizeof(buffer), msg, args);

    va_end(args);

    strcat(buffer, "\n");

    FILE *fp = fopen(log_dir.c_str(), "a");
    if (fp != NULL)
    {
        fprintf(fp, "%s", buffer);
        fclose(fp);
    }
    else
    {
        printf("Failed to open log file.\n");
    }
}

#endif