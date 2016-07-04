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

#include "include/rotoSolver/baseDefs.hpp"

#ifdef __GNUC__
#include <cxxabi.h>
#endif

namespace debug
{
	ostream& SupportLocalDebug::__localDebug(const char *file, const int line, const char *function) const
	{
#ifdef __GNUC__
		string msg = string(file) + "(" + errors::to_string(line) + "): " + abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0) + string(": ") + string(function) + string("(): ");
#else
		string msg = string(file) + "(" + errors::to_string(line) + "): " + string(typeid(*this).name()) + string(": ") + string(function) + string("(): ");
#endif
		dbg << msg;
		return dbg;
	}

	ostream& StaticSupportLocalDebug::__localDebug(const char *file, const int line, const char *function)
	{
#ifdef __GNUC__
		string msg = string(file) + "(" + errors::to_string(line) + "): " + /*abi::__cxa_demangle(typeid(*this).name(), 0, 0, 0) +*/ string(": ") + string(function) + string("(): ");
#else
		string msg = string(file) + "(" + errors::to_string(line) + "): " + string(typeid(*this).name()) + string(": ") + string(function) + string("(): ");
#endif
		dbg << msg;
		return dbg;
	}

	ostream& __functionDebug(const char* file, const int line, const char* function)
	{
		string msg = string(file) + "(" + errors::to_string(line) + "): Function: " + string(function) + string("(): ");
		dbg << msg;
		return dbg;
	}
}

void displayExceptionError(exception& e, string errorMsg)
{
	if (errorMsg.length() > 0)
        std::cerr << errorMsg << std::endl;
    std::cerr << "Caught exception (" << e.what() << ")" << std::endl;
}
