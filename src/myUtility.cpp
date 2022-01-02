#include <string>
#include <time.h>

const std::string getCurrentTime(const std::string nameHeader)
{
    //https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
    time_t now = time(0);
    struct tm timeStruct;
    char buf[80];
    timeStruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &timeStruct);

    //https://marycore.jp/prog/cpp/convert-char-to-string/
    std::string output(buf, 19);
    output = nameHeader + output;
    return output;
}
