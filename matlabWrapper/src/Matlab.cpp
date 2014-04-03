#include "matlab_wrapper/Matlab.h"

#include <vector>

Engine* smartgeometry::Matlab::matlabEngine_ = NULL;

bool
smartgeometry::Matlab::runCode( std::vector<MatlabVariable>     const& _ins
                                , std::vector<MatlabVariable>        & _outs
                                , std::string                   const& _code )
{
    if (!openEngine())
    {
        std::cerr << "Failed to run Matlab code!\n";
        return false;
    }

    std::vector<mxArray*> inVarArrays;

    std::vector<MatlabVariable>::const_iterator itInVars ( _ins.begin()  ), inVarsEnd(_ins.end());
    std::vector<MatlabVariable>::iterator       itOutVars( _outs.begin() ), outVarsEnd(_outs.end());

    for( ; itInVars!=inVarsEnd; ++itInVars )
    {
        // Create a 0-by-0 mxArray; will allocate the memory dynamically
        mxArray* cInVar = 0;

        if (itInVars->type_ == mxDOUBLE_CLASS)
        {
            cInVar = mxCreateNumericMatrix(0, 0, mxDOUBLE_CLASS, mxREAL);

            // Now allocate some memory to copy the values from this in variable so we can afterwards destroy the mxarray (and this memory) without hurting the initial in variable's data_ field
            double* cInData = (double*) mxMalloc( itInVars->nRows_ * itInVars->nColumns_ * sizeof(double));

            for (int i=0; i < itInVars->nRows_ * itInVars->nColumns_ ; i++)
            {
                cInData[i] = itInVars->data_[i];
            }

            // Put the C++ array into the mxArray and define its dimensions
            mxSetPr(cInVar, cInData);
            mxSetM(cInVar, itInVars->nRows_); //number of rows
            mxSetN(cInVar, itInVars->nColumns_); //number of columns
        }

        if (itInVars->type_ == mxCHAR_CLASS)
        {
            cInVar = mxCreateString(itInVars->cdata_);
        }

        engPutVariable( matlabEngine_, itInVars->name_.c_str(), cInVar);
        inVarArrays.push_back(cInVar);
    }

    std::string codeCopy(_code);
    codeCopy += " ";

    // Seems a bit unnecessary to add this extra code just to get the size of each out variable, but lets try it
    // Update: You don't really need these, extra variables just to store the size of the out vars, there is the mxGetM and mxGetN calls (didn't know at the time)
    for( ; itOutVars!=outVarsEnd; ++itOutVars)
    {
        std::string sizeName = itOutVars->name_ + "_size";

        std::string addedCode = sizeName + " = size(" + itOutVars->name_ + "); ";

        codeCopy += addedCode;
    }

    // Evaluate the code
    engEvalString(matlabEngine_, codeCopy.c_str());

    itOutVars = _outs.begin();
    // Get the values for each out variable
    for( ; itOutVars!=outVarsEnd; ++itOutVars)
    {
        std::string sizeName = itOutVars->name_ + "_size";

        mxArray* cOutSizeVar = engGetVariable(matlabEngine_,sizeName.c_str());

        double* cOutSizeData = mxGetPr(cOutSizeVar);

        if(!cOutSizeData)
        {
            std::cerr << "Error while trying to get the size of " << itOutVars->name_.c_str() << ". Variable containing the size seems to be NULL!\n" ;
            continue;
        }
        // Hopefully cOutSizeData will contain the number of rows and columns of the specified out variable
        int nrows = cOutSizeData[0];
        int ncols = cOutSizeData[1];

        itOutVars->nRows_ = nrows;
        itOutVars->nColumns_ = ncols;

        mxArray* cOutVar = engGetVariable(matlabEngine_,itOutVars->name_.c_str());

        if (itOutVars->type_ == mxDOUBLE_CLASS )
        {
            double* cOutData = mxGetPr( cOutVar );

            // The out variable's data_ field should not point to any data, but just is case it does, delete it and give a message
            if ( itOutVars->data_.size() != nrows * ncols )
            {
                std::cout << "Out variable's " << itOutVars->name_.c_str() << " data field seems to point to some memory although it shouldn't. Deallocating this memory!" ;
                itOutVars->data_.resize( nrows * ncols );
                //delete [](itOutVars->data_);
            }
            std::copy( cOutData, cOutData + nrows * ncols * sizeof(double), itOutVars->data_.begin() );

            // Now allocate some memory to put the values for this out variable and copy the values from the mxarray so we can then destroy it
            //itOutVars->data_ = new double[nrows*ncols]; //*sizeof(double));

//            for (int i=0; i<nrows*ncols; i++)
//            {
//                itOutVars->data_[i] = cOutData[i];
//            }
        }

        if (itOutVars->type_ == mxCHAR_CLASS)
        {
            char* cOutData = mxArrayToString(cOutVar);
            // The out variable's cdata_ field should not point to any data, but just is case it does, delete it and give a message
            if(itOutVars->cdata_)
            {
                std::cout << "Out variable's " << itOutVars->name_.c_str() << " data field seems to point to some memory although it shouldn't. Deallocating this memory!" ;

                delete [](itOutVars->cdata_);
            }

            itOutVars->cdata_ = new char[nrows*ncols + 1];

            for (int i=0; i<nrows*ncols; i++)
            {
                itOutVars->cdata_[i] = cOutData[i];
            }
            itOutVars->cdata_[nrows*ncols] = '\0';
        }

        // Done copying, we can now destroy the mxarray for the out variable as well as the mxarray for the out variable's size
        mxDestroyArray(cOutVar);
        mxDestroyArray(cOutSizeVar);
    }

    // Go over the allocated inVarArrays and destroy them. This should also delete the memory associated with every array (all the cInData's) otherwise we will have memory leaks
    std::vector<mxArray*>::iterator itInVarArrays(inVarArrays.begin()), inVarArraysEnd(inVarArrays.end());

    for( ; itInVarArrays!=inVarArraysEnd; ++itInVarArrays)
    {
        mxDestroyArray(*itInVarArrays);
    }

    return true;
}
