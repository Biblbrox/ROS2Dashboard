#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <sstream>

#include "StrUtils.hpp"

namespace ros2monitor {

std::vector<std::string> loadLinesTxt(const std::string &fileName)
{
    std::vector<std::string> res;
    std::ifstream in(fileName, std::ios::in);
    std::string line;
    while (std::getline(in, line))
        res.push_back(line);

    return res;
}

std::vector<std::string> split(const std::string &str, const std::string &del)
{
    std::string s(str);
    size_t pos = 0;
    std::vector<std::string> res;
    while (s.rfind(del, 0) == 0)
        s.erase(0, del.length());
    while ((pos = s.find(del)) != std::string::npos) {
        res.emplace_back(s.substr(0, pos));
        s.erase(0, pos + del.length());
    }
    res.emplace_back(s);

    return res;
}

std::string readFile(const std::string &file)
{
    if (!std::filesystem::exists(file))
        throw std::invalid_argument(fmt::format("File {} doesn't exists", file));

    std::ifstream in(file, std::ios::in);
    std::stringstream stringstream;
    stringstream << in.rdbuf();

    return stringstream.str();
}

std::size_t replaceAll(std::string &inout, std::string_view what,
                                           std::string_view with)
{
    std::size_t count{};
    for (std::string::size_type pos{};
         inout.npos != (pos = inout.find(what.data(), pos, what.length()));
         pos += with.length(), ++count) {
        inout.replace(pos, what.length(), with.data(), with.length());
    }
    return count;
}

std::string u16strTou8str(const std::u16string &str)
{
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> convert;
    std::string s = convert.to_bytes(str);

    return s;
}

std::string unicodeToUtf8(int unicode)
{
    std::string s;

    if (unicode >= 0 and unicode <= 0x7f)// 7F(16) = 127(10)
    {
        s = static_cast<char>(unicode);

        return s;
    } else if (unicode <= 0x7ff)// 7FF(16) = 2047(10)
    {
        unsigned char c1 = 192, c2 = 128;

        for (int k = 0; k < 11; ++k) {
            if (k < 6)
                c2 |= (unicode % 64) & (1 << k);
            else
                c1 |= (unicode >> 6) & (1 << (k - 6));
        }

        s = c1;
        s += c2;

        return s;
    } else if (unicode <= 0xffff)// FFFF(16) = 65535(10)
    {
        unsigned char c1 = 224, c2 = 128, c3 = 128;

        for (int k = 0; k < 16; ++k) {
            if (k < 6)
                c3 |= (unicode % 64) & (1 << k);
            else if (k < 12)
                c2 |= (unicode >> 6) & (1 << (k - 6));
            else
                c1 |= (unicode >> 12) & (1 << (k - 12));
        }

        s = c1;
        s += c2;
        s += c3;

        return s;
    } else if (unicode <= 0x1fffff)// 1FFFFF(16) = 2097151(10)
    {
        unsigned char c1 = 240, c2 = 128, c3 = 128, c4 = 128;

        for (int k = 0; k < 21; ++k) {
            if (k < 6)
                c4 |= (unicode % 64) & (1 << k);
            else if (k < 12)
                c3 |= (unicode >> 6) & (1 << (k - 6));
            else if (k < 18)
                c2 |= (unicode >> 12) & (1 << (k - 12));
            else
                c1 |= (unicode >> 18) & (1 << (k - 18));
        }

        s = c1;
        s += c2;
        s += c3;
        s += c4;

        return s;
    } else if (unicode <= 0x3ffffff)// 3FFFFFF(16) = 67108863(10)
    {
        ;// actually, there are no 5-bytes unicodes
    } else if (unicode <= 0x7fffffff)// 7FFFFFFF(16) = 2147483647(10)
    {
        ;// actually, there are no 6-bytes unicodes
    } else
        ;// incorrect unicode (< 0 or > 2147483647)

    return "";
}

char16_t utf8ToUnicode(std::string utf8_code)
{
    unsigned utf8_size = utf8_code.length();
    char16_t unicode = 0;

    for (unsigned p = 0; p < utf8_size; ++p) {
        int bit_count = (p ? 6 : 8 - utf8_size - (utf8_size == 1 ? 0 : 1)),
            shift = (p < utf8_size - 1 ? (6 * (utf8_size - p - 1)) : 0);

        for (int k = 0; k < bit_count; ++k)
            unicode += ((utf8_code[p] & (1 << k)) << shift);
    }

    return unicode;
}

bool endsWith(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}