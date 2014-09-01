#include "smartgeometry/util.h"



namespace smartgeometry
{
    namespace util
    {
        std::string timestamp2Str( std::string prefix )
        {
            time_t rawtime;
            struct tm * timeinfo;
            char buffer [80];

            time ( &rawtime );
            timeinfo = localtime (&rawtime);

            strftime ( buffer, 80, "_%Y%m%d_%H%M", timeinfo );

            return prefix + buffer;
        }
    }
}
