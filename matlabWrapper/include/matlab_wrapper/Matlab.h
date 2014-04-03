#ifndef __SG_MATLAB_H__
#define __SG_MATLAB_H__

#include <vector>
#include <string>
#include <iostream>
#include "engine.h"
#include "matrix.h"

namespace smartgeometry
{

    // Define MATLAB_APP_PATH somewhere because it will need to call it
    class Matlab
    {
        public:
            struct MatlabVariable
            {
                    mxClassID    type_;
                    int          nRows_;
                    int          nColumns_;
                    std::string  name_;
                    std::vector<double> data_;
                    // Used to be only data_, added cdata_ later when we need to pass a string
                    // Obviously cdata_ won't be used if you don't need a string variable
                    char         *cdata_;

                    MatlabVariable()
                        : type_    ( mxDOUBLE_CLASS )
                        , nRows_   ( 0 )
                        , nColumns_( 0 )
                        , name_    ( "" )
                        //, data_    ( NULL )
                        , cdata_   ( NULL ) {}

                    MatlabVariable( int rows, int columns, std::string name, std::vector<double> data = {} )
                        : type_    ( mxDOUBLE_CLASS )
                        , nRows_   ( rows )
                        , nColumns_( columns )
                        , name_    ( name )
                        //, data_    ( NULL )
                        , cdata_   ( NULL )
                    {
                        //data_ = new double[ rows * columns ];
                        data_.insert( data_.begin(), data.begin(), data.end() );
                    }

                    virtual ~MatlabVariable()
                    {
                        //if ( data_  ) { delete [] (data_ );  data_ = 0; }
                        if ( cdata_ ) { delete [] (cdata_); cdata_ = 0; }
                    }


            };

            static void init()
            {
                matlabEngine_ = 0;
            }

            static bool openEngine()
            {
                // Open Matlab in case it was not opened
                if ( !Matlab::matlabEngine_ )
                {
                    std::cout << "calling engOpen(" << (MATLAB_APP_PATH) << ")\n";
                    if ( !(Matlab::matlabEngine_ = engOpen(MATLAB_APP_PATH)) )
                    {
                        std::cerr << "Can't start MATLAB engine\n" ;
                        return false;
                    }
                }
                else
                {
                    //engEvalString(matlabEngine_, "clear all");
                }
            }

            static void closeEngine()
            {
                if(matlabEngine_)
                {
                    engClose(matlabEngine_);
                }
            }

            static bool runCode( std::vector<MatlabVariable>     const& _ins
                                 , std::vector<MatlabVariable>        & _outs
                                 , std::string                   const& _code );

        protected:
            // Probably there's a better way to do this
            static Engine* matlabEngine_;
            Matlab(){}
    };
}

#endif // __SG_MATLAB_H__
