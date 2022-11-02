#pragma once

#include "pure_pursuit/pure_pursuit.h"

namespace control {
class AdaptivePurePursuit final : public PurePursuit {
public:
  explicit AdaptivePurePursuit(const Config &config) : config_(config){};

private:
  Config config_;
};
} // namespace control