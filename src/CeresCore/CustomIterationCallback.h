#pragma once
#include "ceres/ceres.h"

namespace CeresCore
{
    typedef void tProgressDelegate();
    typedef tProgressDelegate* ProgressDelegate;

    class CustomIterationCallback :
        public ceres::IterationCallback
    {
        

    public:

        ~CustomIterationCallback()
        {

        }
        virtual ceres::CallbackReturnType operator()(const
            ceres::IterationSummary& summary) 
        {
            _iterSummary = summary;
            /*std::cout << "Callback executed" << std::endl;*/
            func();
            /*std::cout << "Function executed" << std::endl;*/
            return _returnType;
        }
        void set_progress_delegate(ProgressDelegate del)
        {
            func = del;
        }

        void set_return_type(ceres::CallbackReturnType& type)
        {
            _returnType = type;
        }

        ceres::IterationSummary* iterSummary() { return &_iterSummary; }
    private:
        ProgressDelegate func;
        ceres::CallbackReturnType _returnType;
        ceres::IterationSummary _iterSummary;
    };
}
