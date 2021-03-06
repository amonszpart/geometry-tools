#ifndef __SG_UTIL_H__
#define __SG_UTIL_H__

#include <string>

namespace smartgeometry
{
    namespace util
    {
        inline std::string timestamp2Str( std::string prefix = "")
        {
            time_t rawtime;
            struct tm * timeinfo;
            char buffer [80];

            time ( &rawtime );
            timeinfo = localtime (&rawtime);

            strftime ( buffer, 80, "_%Y%m%d_%H%M", timeinfo );

            return prefix + buffer;
        }
    } // ... ns util
} // ... ns smartgeometry

#endif // __SG_UTIL_H__
