#ifndef STRING_H
#define STRING_H

//c++11 no longer supplies a strcasecmp, so define our own version.
static inline int stringcasecompare(const char* a, const char* b)
{
    while(*a && *b)
    {
        if (tolower(*a) != tolower(*b))
            return tolower(*a) - tolower(*b);
        a++;
        b++;
    }
    return *a - *b;
}

#endif//STRING_H
