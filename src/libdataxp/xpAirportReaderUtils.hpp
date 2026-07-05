//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <algorithm>
#include <cctype>
#include <functional>
#include <string>
#include <vector>

namespace world
{
    inline void parseSeparatedList(
        const std::string& listText,
        const std::string& delimiters,
        std::function<void(const std::string& item)> parseItem)
    {
        int lastDelimiterIndex = -1;

        for (int i = 0 ; i < listText.length(); i++)
        {
            char c = listText[i];
            bool isDelimiter = (delimiters.find(c) != std::string::npos);
            if (isDelimiter)
            {
                if (i > lastDelimiterIndex + 1)
                {
                    std::string itemText = listText.substr(lastDelimiterIndex + 1, i - lastDelimiterIndex - 1);
                    parseItem(itemText);
                }
                lastDelimiterIndex = i;
            }
        }

        if (lastDelimiterIndex < (int)listText.length() - 1)
        {
            std::string itemText = listText.substr(lastDelimiterIndex + 1, listText.length() - lastDelimiterIndex - 1);
            parseItem(itemText);
        }
    }

    inline std::string trimCopy(const std::string& value)
    {
        const auto begin = value.find_first_not_of(" \t\r\n");
        if (begin == std::string::npos)
        {
            return "";
        }

        const auto end = value.find_last_not_of(" \t\r\n");
        return value.substr(begin, end - begin + 1);
    }

    inline std::string toUpperCopy(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::toupper(c));
        });
        return value;
    }

    inline bool equalsIgnoreCase(const std::string& left, const std::string& right)
    {
        return toUpperCopy(left) == toUpperCopy(right);
    }

    inline bool tokenListContains(const std::string& listText, const std::string& token)
    {
        const std::string expected = toUpperCopy(token);
        bool found = false;
        parseSeparatedList(listText, ",;:| 	", [&found, &expected](const std::string& item) {
            if (toUpperCopy(trimCopy(item)) == expected)
            {
                found = true;
            }
        });
        return found;
    }

    inline std::vector<std::string> splitTokens(const std::string& lineText)
    {
        std::vector<std::string> tokens;
        parseSeparatedList(lineText, " 	", [&tokens](const std::string& item) {
            const std::string trimmed = trimCopy(item);
            if (!trimmed.empty())
            {
                tokens.push_back(trimmed);
            }
        });
        return tokens;
    }

    inline void appendUniqueText(std::vector<std::string>& items, const std::string& value)
    {
        if (!value.empty() && std::find(items.begin(), items.end(), value) == items.end())
        {
            items.push_back(value);
        }
    }

    inline std::string normalizeLookupText(const std::string& value)
    {
        std::string normalized;
        normalized.reserve(value.size());

        bool previousWasSpace = true;
        for (const auto ch : value)
        {
            if (std::isalnum(static_cast<unsigned char>(ch)))
            {
                normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
                previousWasSpace = false;
            }
            else if (!previousWasSpace)
            {
                normalized.push_back(' ');
                previousWasSpace = true;
            }
        }

        while (!normalized.empty() && normalized.back() == ' ')
        {
            normalized.pop_back();
        }

        return normalized;
    }
}
