#ifndef SHARE_TYPES_H
#define SHARE_TYPE_H

#include <functional>

// Tracking state callback
// -1 - left; 0 - center; 1 - right; 2 - lose
using LineCallback = std::function<void(int)>;

// distance measurement complete callback
// distance - cm
using DistanceCallback = std::function<void(float)>;

#endif