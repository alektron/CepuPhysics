#pragma once

#include <vector>
#include <stdint.h>
#include <cassert>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "UtilitiesForward.h"
#include "Memory/Buffer.h"
#include "Memory/BufferPool.h"

#include "BoundingBox.h"
#include "Collidables/TypedIndex.h"

const static bool CONSTRAINTS_UNSUPPORTED = true;
const static bool SLEEPING_UNSUPPORTED = true;
const static bool SOLVER_UNSUPPORTED = true;
const static bool MULTITHREADING_UNSUPPORTED = true;
