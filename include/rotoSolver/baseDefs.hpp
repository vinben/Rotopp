/**************************************************************************
** This file is a part of our work (Siggraph'16 paper, binary, code and dataset):
**
** Roto++: Accelerating Professional Rotoscoping using Shape Manifolds
** Wenbin Li, Fabio Viola, Jonathan Starck, Gabriel J. Brostow and Neill D.F. Campbell
**
** w.li AT cs.ucl.ac.uk
** http://visual.cs.ucl.ac.uk/pubs/rotopp
**
** Copyright (c) 2016, Wenbin Li
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** -- Redistributions of source code and data must retain the above
**    copyright notice, this list of conditions and the following disclaimer.
** -- Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** THIS WORK AND THE RELATED SOFTWARE, SOURCE CODE AND DATA IS PROVIDED BY
** THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
** WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
** NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
** INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
** BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
** USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
** NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#ifndef BASEDEFS_H
#define BASEDEFS_H

#include <cassert>
#include <memory>
#include <iostream>

using std::shared_ptr;
using std::cout;
using std::endl;
using std::ostream;
using std::exception;

#define nassert(e)  \
    ((void) ((e) ? ((void)0) : __nassert (#e, __FILE__, __LINE__)))
#define __nassert(e, file, line) \
    ((void)printf ("%s:%u: failed assertion `%s'\n", file, line, e), abort())

#ifndef vdbg
#define vdbg(var) std::cout << #var << " = " << (var) << std::endl;
#endif

#define dbg std::cout

#define UNUSED_PARAMETER (void)

#ifdef __APPLE__
typedef unsigned int uint;
#endif

/* Special function added to classes for error reporting. Implement a member class
  by implementing __localError(string reason, const char* file, const int line).
  Can do this by inheriting the following class:
  */
#define localError(reason) __localError((reason), __FILE__, __LINE__, __func__)

#define functionError(reason) errors::__functionError((reason), __FILE__, __LINE__, __func__)

#include <sstream>
using std::string;
using std::stringstream;

namespace errors
{
    template <typename T> inline string to_string (const T& t)
    {
        std::stringstream ss;
        ss << t;
        return ss.str();
    };
}

#define localDebug __localDebug(__FILE__, __LINE__, __func__)

#define functionDebug debug::__functionDebug(__FILE__, __LINE__, __func__)

#ifdef __GNUC__
#include <cxxabi.h>
#endif

namespace debug
{
    template<class T> void dbg_print_type(T)
    {
#ifdef __GNUC__
        dbg << abi::__cxa_demangle(typeid(T).name(), 0, 0, 0) << endl;
#else
        dbg << string(typeid(T).name()) << endl;
#endif
    }

    class SupportLocalDebug
    {
    protected:
        virtual ostream& __localDebug(const char* file, const int line, const char* function) const;
    };

    class StaticSupportLocalDebug
    {
    protected:
        static ostream& __localDebug(const char* file, const int line, const char* function);
    };

    ostream& __functionDebug(const char* file, const int line, const char* function);

    struct DoNothing
    {
        // See helpful answer on stack overflow:
        // http://stackoverflow.com/questions/1134388/stdendl-is-of-unknown-type-when-overloading-operator
        //
        static DoNothing& getDoNothing()
        {
            static DoNothing d;
            return d;
        }

        DoNothing()
        {}

        template <typename T>
        DoNothing& operator<< (const T& t)
        {
            return *this;
        }

        typedef DoNothing& (*DoNothingManipulator)(DoNothing&);

        DoNothing& operator<< (DoNothingManipulator manip)
        {
            return manip(*this);
        }

        static DoNothing& endl(DoNothing& d)
        {
            return d;
        }

        typedef std::basic_ostream<char, std::char_traits<char> > CoutType;

        typedef CoutType& (*StandardEndLine)(CoutType&);

        DoNothing& operator << (StandardEndLine manip)
        {
            (void)manip;
            return *this;
        }
    };
}

void displayExceptionError(exception& e, string errorMsg = "");

template<typename T> inline T sgn(T v)
{
    return (v < 0) ? T(-1) : T(1);
}

#endif // BASEDEFS_H
