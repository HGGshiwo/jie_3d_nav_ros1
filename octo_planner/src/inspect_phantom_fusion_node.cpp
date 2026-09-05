#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <memory>
#include <iostream>
#include <iomanip>
#include <vector>

struct SubPoint {
  std::string name;
  octomap::point3d pt;
};

class InspectPhantomFusionNode {
public:
  InspectPhantomFusionNode() : nh_("~") {
    nh_.param<double>("target_x", target_x_, 1.4);
    nh_.param<double>("target_y", target_y_, 0.2);
    nh_.param<double>("target_z", target_z_, 0.1);
    nh_.param<std::string>("global_topic", global_topic_, "/octomap_global");
    nh_.param<std::string>("local_topic", local_topic_, "/octomap_local");
    nh_.param<std::string>("fused_topic", fused_topic_, "/octomap_fused");
    nh_.param<double>("crop_radius_xy", crop_radius_xy_, 3.0);
    nh_.param<double>("crop_height_above", crop_height_above_, 2.0);
    nh_.param<double>("crop_height_below", crop_height_below_, 1.0);

    global_sub_ = nh_.subscribe(global_topic_, 1, &InspectPhantomFusionNode::onGlobalMsg, this);
    local_sub_ = nh_.subscribe(local_topic_, 1, &InspectPhantomFusionNode::onLocalMsg, this);
    fused_sub_ = nh_.subscribe(fused_topic_, 1, &InspectPhantomFusionNode::onFusedMsg, this);

    ROS_INFO_STREAM("InspectPhantomFusionNode initialized for target center ["
                    << target_x_ << ", " << target_y_ << ", " << target_z_ << "]");
  }

  void onGlobalMsg(const octomap_msgs::Octomap::ConstPtr& msg) {
    global_msg_ = msg;
    triggerInspectionIfReady();
  }

  void onLocalMsg(const octomap_msgs::Octomap::ConstPtr& msg) {
    local_msg_ = msg;
    triggerInspectionIfReady();
  }

  void onFusedMsg(const octomap_msgs::Octomap::ConstPtr& msg) {
    fused_msg_ = msg;
    triggerInspectionIfReady();
  }

  void triggerInspectionIfReady() {
    if (inspected_) return;
    if (!global_msg_ || !local_msg_ || !fused_msg_) return;

    inspected_ = true;
    runDiagnosticTrace();
  }

  void runDiagnosticTrace() {
    std::cout << "\n======================================================================\n";
    std::cout << " 🔍 八叉树拓扑深度透视诊断报告\n";
    std::cout << " 中心参照坐标: [" << target_x_ << ", " << target_y_ << ", " << target_z_ << "]\n";
    std::cout << "======================================================================\n";

    std::unique_ptr<octomap::OcTree> global_tree;
    std::unique_ptr<octomap::OcTree> local_tree;
    std::unique_ptr<octomap::OcTree> fused_topic_tree;

    if (global_msg_) {
      octomap::AbstractOcTree* abs = octomap_msgs::msgToMap(*global_msg_);
      if (abs) global_tree.reset(dynamic_cast<octomap::OcTree*>(abs));
    }
    if (local_msg_) {
      octomap::AbstractOcTree* abs = octomap_msgs::msgToMap(*local_msg_);
      if (abs) local_tree.reset(dynamic_cast<octomap::OcTree*>(abs));
    }
    if (fused_msg_) {
      octomap::AbstractOcTree* abs = octomap_msgs::msgToMap(*fused_msg_);
      if (abs) fused_topic_tree.reset(dynamic_cast<octomap::OcTree*>(abs));
    }

    octomap::point3d target_pt(target_x_, target_y_, target_z_);

    // 核心修正：取每个 0.20m 体素真正的几何中心，避开 z=0.0, 0.2 的分界线
    // 下层中心 z = 0.10，上层中心 z = 0.30
    std::vector<SubPoint> sub_pts = {
      {"下层1 (左下)", octomap::point3d(target_x_ - 0.1, target_y_ - 0.1, 0.10)},
      {"下层2 (右下)", octomap::point3d(target_x_ + 0.1, target_y_ - 0.1, 0.10)},
      {"下层3 (左上)", octomap::point3d(target_x_ - 0.1, target_y_ + 0.1, 0.10)},
      {"下层4 (右上)", octomap::point3d(target_x_ + 0.1, target_y_ + 0.1, 0.10)},
      {"上层5 (左下)", octomap::point3d(target_x_ - 0.1, target_y_ - 0.1, 0.30)},
      {"上层6 (右下)", octomap::point3d(target_x_ + 0.1, target_y_ - 0.1, 0.30)},
      {"上层7 (左上)", octomap::point3d(target_x_ - 0.1, target_y_ + 0.1, 0.30)},
      {"上层8 (右上)", octomap::point3d(target_x_ + 0.1, target_y_ + 0.1, 0.30)}
    };

    std::cout << "\n【1. 真实节点类型透视 (判断到底是不是叶子)】\n";
    inspectNodeStructure("全局静态 Topic (/octomap_global)", global_tree.get(), target_pt);
    inspectNodeStructure("局部动态 Topic (/octomap_local)", local_tree.get(), target_pt);
    inspectNodeStructure("当前融合 Topic (/octomap_fused)", fused_topic_tree.get(), target_pt);

    std::cout << "\n【2. 8 个子方块真实几何中心点探测】\n";
    printSubPoints("融合 Topic 8个几何中心探测", fused_topic_tree.get(), sub_pts);

    std::cout << "\n【3. 内存逐步推演 (观察剪枝前后变化)】\n";
    simulateROIChain(local_tree.get(), global_tree.get(), sub_pts, target_pt);

    std::cout << "======================================================================\n\n";
  }

private:
  void inspectNodeStructure(const std::string& label, octomap::OcTree* tree, const octomap::point3d& p) {
    std::cout << "  - " << label << " 在 [" << p.x() << ", " << p.y() << ", " << p.z() << "]:\n";
    if (!tree) {
      std::cout << "      [Tree 未就绪]\n";
      return;
    }

    // 1. 获取 search() 命中的真实节点及其深度
    unsigned depth = 0;
    octomap::OcTreeNode* search_node = searchNodeWithDepth(tree, p, depth);
    if (!search_node) {
      std::cout << "      search() 结果: 未分配 (NULL)\n";
    } else {
      bool has_children = tree->nodeHasChildren(search_node);
      double node_size = tree->getNodeSize(depth);
      std::cout << "      search() 命中节点: 深度=" << depth 
                << " (尺寸=" << std::fixed << std::setprecision(2) << node_size << "m)"
                << ", 类别=" << (has_children ? "【内部父节点 (Inner Node)】" : "【叶子节点 (Leaf Node)】")
                << ", LogOdds=" << std::setprecision(3) << search_node->getLogOdds()
                << ", 状态=" << (tree->isNodeOccupied(search_node) ? "Occupied" : "Free") << "\n";

      if (has_children) {
        int allocated_children = 0;
        for (unsigned int i = 0; i < 8; ++i) {
          if (tree->nodeChildExists(search_node, i)) allocated_children++;
        }
        std::cout << "        -> 该父节点下已分配子节点数: " << allocated_children << " / 8\n";
      }
    }

    // 2. 打印 leaf_bbx_iterator 命中的真实叶节点
    int leaf_count = 0;
    for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(p, p), end = tree->end_leafs_bbx(); it != end; ++it) {
      leaf_count++;
      std::cout << "      leaf_bbx 命中的真实叶子 #" << leaf_count 
                << ": 中心=[" << it.getCoordinate().x() << "," << it.getCoordinate().y() << "," << it.getCoordinate().z() << "]"
                << ", 尺寸=" << it.getSize() << "m"
                << ", LogOdds=" << it->getLogOdds() << "\n";
      break; 
    }
  }

  octomap::OcTreeNode* ensureNodeExpandedToMaxDepth(octomap::OcTree* tree, const octomap::OcTreeKey& key) {
    if (!tree) return nullptr;
    octomap::OcTreeNode* cur = tree->getRoot();
    if (!cur) return nullptr;

    const unsigned int max_depth = tree->getTreeDepth();

    for (unsigned int d = max_depth; d > 0; --d) {
      if (!tree->nodeHasChildren(cur)) {
        if (tree->isNodeOccupied(cur)) {
          return cur;
        }
        tree->expandNode(cur);
      }

      unsigned int pos = 0;
      if (key[0] & (1 << (d - 1))) pos |= 1;
      if (key[1] & (1 << (d - 1))) pos |= 2;
      if (key[2] & (1 << (d - 1))) pos |= 4;

      if (!tree->nodeChildExists(cur, pos)) {
        return nullptr;
      }

      cur = tree->getNodeChild(cur, pos);
    }

    return cur;
  }

  octomap::OcTreeNode* searchNodeWithDepth(octomap::OcTree* tree, const octomap::point3d& p, unsigned& depth) {
    octomap::OcTreeKey key;
    if (!tree->coordToKeyChecked(p, key)) return nullptr;

    octomap::OcTreeNode* cur = tree->getRoot();
    depth = 0;
    if (!cur) return nullptr;

    for (unsigned int d = tree->getTreeDepth(); d > 0; --d) {
      if (!tree->nodeHasChildren(cur)) return cur;
      unsigned int child_idx = computeChildIdx(key, d - 1);
      if (tree->nodeChildExists(cur, child_idx)) {
        cur = tree->getNodeChild(cur, child_idx);
        depth++;
      } else {
        return cur;
      }
    }
    return cur;
  }

  unsigned computeChildIdx(const octomap::OcTreeKey& key, unsigned int depth) {
    unsigned int pos = 0;
    if (key[0] & (1 << depth)) pos |= 1;
    if (key[1] & (1 << depth)) pos |= 2;
    if (key[2] & (1 << depth)) pos |= 4;
    return pos;
  }

  void printSubPoints(const std::string& step_label, octomap::OcTree* tree, const std::vector<SubPoint>& sub_pts) {
    std::cout << "  - " << step_label << ":\n";
    for (const auto& sp : sub_pts) {
      unsigned depth = 0;
      octomap::OcTreeNode* node = searchNodeWithDepth(tree, sp.pt, depth);
      if (!node) {
        std::cout << "      " << sp.name << " [" << sp.pt.x() << "," << sp.pt.y() << "," << sp.pt.z() << "]: NULL\n";
      } else {
        double size = tree->getNodeSize(depth);
        bool is_leaf = !tree->nodeHasChildren(node);
        std::cout << "      " << sp.name << " [" << sp.pt.x() << "," << sp.pt.y() << "," << sp.pt.z() << "]: "
                  << (tree->isNodeOccupied(node) ? "Occupied" : "Free")
                  << " (LogOdds=" << std::fixed << std::setprecision(3) << node->getLogOdds()
                  << ", size=" << std::setprecision(2) << size << "m"
                  << ", " << (is_leaf ? "Leaf" : "Inner") << ")\n";
      }
    }
  }

  void simulateROIChain(octomap::OcTree* local_tree, octomap::OcTree* global_tree,
                        const std::vector<SubPoint>& sub_pts, const octomap::point3d& p) {
    if (!local_tree || !global_tree) return;
    std::unique_ptr<octomap::OcTree> fused_sim(new octomap::OcTree(*local_tree));

    std::cout << "\n▶ Step 1 (Local 树克隆后):\n";
    printSubPoints("Step 1", fused_sim.get(), sub_pts);

    // 写入 Global
    const double min_x = p.x() - 1.0;
    const double max_x = p.x() + (crop_radius_xy_ + 1.5);
    const double min_y = p.y() - crop_radius_xy_;
    const double max_y = p.y() + crop_radius_xy_;
    const double min_z = p.z() - crop_height_below_;
    const double max_z = p.z() + crop_height_above_;

    const double target_res = fused_sim->getResolution();
    const double half_target_res = target_res * 0.5;

    for (octomap::OcTree::leaf_bbx_iterator it = global_tree->begin_leafs_bbx(octomap::point3d(min_x, min_y, min_z),
                                                                            octomap::point3d(max_x, max_y, max_z)),
                                            end = global_tree->end_leafs_bbx(); it != end; ++it) {
      if (global_tree->isNodeOccupied(*it)) {
        const float global_log_odds = it->getLogOdds();
        const double node_size = it.getSize();
        const octomap::point3d center = it.getCoordinate();

        if (node_size <= target_res + 1e-6) {
          octomap::OcTreeKey key;
          octomap::OcTreeNode* node = nullptr;
          if (fused_sim->coordToKeyChecked(center, key)) {
            node = ensureNodeExpandedToMaxDepth(fused_sim.get(), key);
          }
          if (!node) {
            node = fused_sim->updateNode(center, 0.0f, true);
          }
          if (node && node->getLogOdds() < global_log_odds) node->setLogOdds(global_log_odds);
        } else {
          const double half_size = node_size * 0.5;
          octomap::point3d cell_min(center.x() - half_size + half_target_res,
                                    center.y() - half_size + half_target_res,
                                    center.z() - half_size + half_target_res);
          octomap::point3d cell_max(center.x() + half_size - half_target_res,
                                    center.y() + half_size - half_target_res,
                                    center.z() + half_size - half_target_res);

          octomap::OcTreeKey min_key, max_key;
          if (fused_sim->coordToKeyChecked(cell_min, min_key) && fused_sim->coordToKeyChecked(cell_max, max_key)) {
            for (auto kx = min_key[0]; kx <= max_key[0]; ++kx) {
              for (auto ky = min_key[1]; ky <= max_key[1]; ++ky) {
                for (auto kz = min_key[2]; kz <= max_key[2]; ++kz) {
                  octomap::OcTreeKey key(kx, ky, kz);
                  octomap::OcTreeNode* node = ensureNodeExpandedToMaxDepth(fused_sim.get(), key);
                  if (!node) {
                    node = fused_sim->updateNode(key, 0.0f, true);
                  }
                  if (node && node->getLogOdds() < global_log_odds) node->setLogOdds(global_log_odds);
                }
              }
            }
          }
        }
      }
    }

    std::cout << "\n▶ Step 2 (写入 Global 障碍物后):\n";
    printSubPoints("Step 2", fused_sim.get(), sub_pts);

    fused_sim->updateInnerOccupancy();
    std::cout << "\n▶ Step 3 (updateInnerOccupancy 之后):\n";
    printSubPoints("Step 3", fused_sim.get(), sub_pts);

    fused_sim->prune();
    std::cout << "\n▶ Step 4 (prune 之后):\n";
    printSubPoints("Step 4", fused_sim.get(), sub_pts);
  }

  ros::NodeHandle nh_;
  ros::Subscriber global_sub_, local_sub_, fused_sub_;
  octomap_msgs::Octomap::ConstPtr global_msg_, local_msg_, fused_msg_;
  double target_x_, target_y_, target_z_;
  std::string global_topic_, local_topic_, fused_topic_;
  double crop_radius_xy_, crop_height_above_, crop_height_below_;
  bool inspected_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "inspect_phantom_fusion");
  InspectPhantomFusionNode node;
  ros::spin();
  return 0;
}