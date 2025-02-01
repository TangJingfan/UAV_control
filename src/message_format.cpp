#include "message_format.h"
#include <Arduino.h>

bool is_target_attitude_format(const String &str) {
  // 1. check sharp bracket format
  if (!str.startsWith("<") || !str.endsWith(">")) {
    return false;
  }

  // 2. check Euler angle format
  if (str.indexOf("y:") == -1 || str.indexOf("p:") == -1 ||
      str.indexOf("r:") == -1) {
    return false;
  }

  return true;
}