// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
/*
#include "core.h"

#if defined( B2_COMPILER_MSVC )
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
#else
#include <stdlib.h>
#endif

#include <stdio.h>
#include <string.h>

#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCAlloc( ptr, size ) TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr ) TracyCFree( ptr )

#else

#define b2TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr )

#endif

#include "atomic.h"
*/
import { b2IsValidFloat } from "../box2d/math_functions.ts";
// This allows the user to change the length units at runtime
// A regular, unexported variable to serve as the "private" backing store
let _b2_lengthUnitsPerMeter: number = 1;

// Export an object that contains the getter and setter
export const b2_lengthUnitsPerMeter = {
  get value(): number {
    return _b2_lengthUnitsPerMeter;
  },

  set value(lengthUnits: number) {
    // Retain the original validation logic
    console.assert(lengthUnits > 0);
    _b2_lengthUnitsPerMeter = lengthUnits;
  },
};

/*
b2Version b2GetVersion( void )
{
	return (b2Version){
		.major = 3,
		.minor = 2,
		.revision = 0,
	};
}
*/