#include <string>
#include <time.h>
#include <iostream>

//--------------------------------------------------------------//
std::string replaceStringSingle(std::string& strToReplace, std::string strBefore, std::string strAfter )
{
    const unsigned int posBefore = strToReplace.find(strBefore);
    const int lenAfter = strAfter.length();

    if(posBefore == std::string::npos || strBefore.empty())
    {
        return strToReplace;
    }

    if(posBefore>=strToReplace.length())
    {
        return strToReplace;
    }

    return strToReplace.replace(posBefore, lenAfter, strToReplace);
}

// String.replace( StartPos, Length, StrAfter)
// Strinf.find( StrFind, StartPos, SearchLength)

//--------------------------------------------------------------//
std::string replaceStringAll(std::string& strToReplace, std::string strBefore, std::string strAfter )
{
    unsigned int posBefore = strToReplace.find(strBefore);
    const int lenBefore = strBefore.length();
    const int lenAfter  = strAfter.length();

    if(strBefore.empty() || strAfter.empty())
    {
        return strToReplace;
    }

    while( (posBefore=strToReplace.find(strBefore, posBefore)) != std::string::npos )
    {
        if(posBefore>=strToReplace.length())
        {
            break;
        }

        strToReplace.replace(posBefore, lenBefore, strAfter);
        posBefore += lenAfter;
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
