#include <hippo_common/param_utils.hpp>

#include "dvl_node.hpp"

namespace dvl {
void DvlNode::InitParams() {
  HIPPO_COMMON_DECLARE_PARAM_READONLY(ip_address);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(port);
}
}  // namespace dvl
