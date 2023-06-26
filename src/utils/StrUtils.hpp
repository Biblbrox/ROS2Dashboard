#pragma once

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace ros2monitor {
using uncstring = std::u16string;

// trim from start (in place)
inline void ltrim(std::string &s)
{
    s.erase(s.begin(),
            std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); })
                    .base(),
            s.end());
}

inline bool isNumber(const std::string &x)
{
    std::regex e("^-?\\d+");
    return std::regex_match(x, e);
}

// trim from both ends (in place)
inline void trim(std::string &s)
{
    rtrim(s);
    ltrim(s);
}

std::size_t replaceAll(std::string &inout, std::string_view what, std::string_view with);

std::string readFile(const std::string &file);

bool endsWith(const std::string &str, const std::string &suffix);


std::string u16strTou8str(const std::u16string &str);

char16_t utf8ToUnicode(std::string utf8_code);

std::string unicodeToUtf8(int unicode);

/**
 * Split lines to vector of words
 * @param str
 * @param del
 * @return
 */
std::vector<std::string> split(const std::string &str, const std::string &del);

/** Load each line from text file
 * @brief loadLinesTxt
 * @param fileName
 * @return
 */
std::vector<std::string> loadLinesTxt(const std::string &fileName);

}
