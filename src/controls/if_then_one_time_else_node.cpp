/* Copyright (C) 2020 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "behaviortree_cpp_v3/controls/if_then_one_time_else_node.h"

namespace BT
{
IfThenOneTimeElseNode::IfThenOneTimeElseNode(const std::string& name) :
  ControlNode::ControlNode(name, {}), child_idx_(0), already_failed(false)
{
  setRegistrationID("IfThenOneTimeElse");
}

void IfThenOneTimeElseNode::halt()
{
  std::cout << "HALT CALLED: Resetting _executed_else" << std::endl;
  child_idx_ = 0;
  already_failed = false;
  ControlNode::halt();
}

NodeStatus IfThenOneTimeElseNode::tick()
{
  const size_t children_count = children_nodes_.size();

  if (children_count != 3)
  {
    throw std::logic_error("IfThenOneTimeElseNode must have 3 children");
  }

  setStatus(NodeStatus::RUNNING);

  if (child_idx_ == 0)
  {
    NodeStatus condition_status = children_nodes_[0]->executeTick();

    if (condition_status == NodeStatus::RUNNING)
    {
      return condition_status;
    }
    else if (condition_status == NodeStatus::SUCCESS)
    {
      child_idx_ = 1;
    }
    else if (condition_status == NodeStatus::FAILURE)
    {
        if (!already_failed) // Execute "Else" only the first time
        {
            already_failed = true;
            child_idx_ = 2;
        }
    }
  }

  if (child_idx_ > 0)
  {
        NodeStatus status = children_nodes_[child_idx_]->executeTick();
        if (status == NodeStatus::RUNNING)
        {
            return NodeStatus::RUNNING;
        }
        else
        {
            resetChildren();
            if (child_idx_ == 1)
                already_failed = false;
            child_idx_ = 0;
            return status;
        }
  }
  else{
    resetChildren();
    return NodeStatus::FAILURE;
  }

  throw std::logic_error("Something unexpected happened in IfThenOneTimeElseNode");
}

}   // namespace BT
