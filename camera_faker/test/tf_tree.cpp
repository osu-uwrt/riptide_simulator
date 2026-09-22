#include "pool_viewer/tf_tree.hpp"
#include <cassert>

int main() {
  pool::TfTree tree;
  std::map<std::string, std::string> parents{
      {"map", ""}, {"gate_frame", "map"}, {"target_frame", "gate_frame"},
      {"talos/base_link", "map"}, {"camera", "talos/base_link"},
      {"disconnected", ""}, {"sensor", "disconnected"}};
  tree.update(parents);
  assert((tree.roots == std::vector<std::string>{"disconnected", "map"}));
  assert(tree.children.at("gate_frame").at(0) == "target_frame");
  assert(tree.frames.at("map").enabled);
  assert(tree.branchSelection("gate_frame") == pool::TfTree::Selection::Shown);
  tree.selectBranch("gate_frame", false);
  assert(tree.branchSelection("gate_frame") == pool::TfTree::Selection::Hidden);
  assert(!tree.frames.at("target_frame").enabled);
  assert(tree.frames.at("talos/base_link").enabled);
  // Individual visibility does not depend on the parent being visible.
  tree.frames.at("target_frame").enabled = true;
  assert(!tree.frames.at("gate_frame").enabled);
  assert(tree.branchSelection("gate_frame") == pool::TfTree::Selection::Mixed);
  // A parent's own checkbox must not change its descendants, and selecting
  // a whole branch must cover grandchildren even when their rows are collapsed.
  parents["grandchild"] = "target_frame";
  tree.update(parents);
  tree.selectBranch("gate_frame", true);
  tree.frames.at("gate_frame").enabled = false;
  assert(tree.frames.at("target_frame").enabled);
  assert(tree.frames.at("grandchild").enabled);
  assert(tree.branchSelection("gate_frame") == pool::TfTree::Selection::Mixed);
  tree.selectBranch("gate_frame", false);
  assert(!tree.frames.at("grandchild").enabled);
  tree.frames.at("gate_frame").enabled = true;
  assert(!tree.frames.at("target_frame").enabled);
  assert(tree.branchSelection("gate_frame") == pool::TfTree::Selection::Mixed);
  tree.frames.at("gate_frame").enabled = false;
  tree.frames.at("gate_frame").available = true;
  parents["gate_frame"] = "disconnected";
  tree.update(parents);
  assert(!tree.frames.at("gate_frame").enabled);
  assert(!tree.frames.at("gate_frame").available);
  assert(tree.children.at("disconnected").at(0) == "gate_frame");
  tree.selectAll(false);
  parents["new_frame"] = "map";
  tree.update(parents);
  assert(!tree.frames.at("new_frame").enabled);
  tree.selectAll(true);
  tree.frames.at("sensor").enabled = false;
  parents.erase("sensor");
  tree.update(parents);
  parents["sensor"] = "map";
  tree.update(parents);
  assert(!tree.frames.at("sensor").enabled);
  parents["cycle_a"] = "cycle_b";
  parents["cycle_b"] = "cycle_a";
  tree.update(parents);
  tree.selectBranch("cycle_a", false);
  assert(!tree.frames.at("cycle_b").enabled);
  assert(tree.roots.front() == "cycle_a");
}
