#include <string>
#include <time.h>

//--------------------------------------------------------------//
std::string replaceStringSingle(std::string& strToReplace, std::string strBefore, std::string strAfter )
{
    const unsigned int pos = strToReplace.find(strBefore);
    const int lenAfter = strAfter.length();

    if(pos == std::string::npos || strBefore.empty())
    {
        return strToReplace;
    }

    return strToReplace.replace(pos, lenAfter, strToReplace);
}

//--------------------------------------------------------------//
std::string replaceStringAll(std::string& strToReplace, std::string strBefore, std::string strAfter )
{
    unsigned int pos = strToReplace.find(strBefore);
    const int lenAfter = strAfter.length();

    if(strBefore.empty())
    {
        return strToReplace;
    }

    while( (pos=strToReplace.find(strBefore, pos)) != std::string::npos )
    {
        strToReplace.replace(pos, strBefore.length(), strAfter);
        pos += lenAfter;
    }

    return strToReplace;
}

//--------------------------------------------------------------//
const std::string getCurrentTime(const std::string nameHeader)
{
    // Get Time
    //https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
    time_t now = time(0);
    struct tm timeStruct;
    char buf[80];
    timeStruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &timeStruct);

    // Add Header
    //https://marycore.jp/prog/cpp/convert-char-to-string/
    std::string output(buf, 19);
    output = nameHeader + output;
    
    // Replace String
    //https://www.sejuku.net/blog/54493
    replaceStringAll(output, ":", "_");

    return output;
}
