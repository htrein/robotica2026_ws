#include "planning_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct TreeNode {
    double x, y, theta;
    TreeNode* parent;
    TreeNode(double x_, double y_, double t_, TreeNode* p = nullptr)
        : x(x_), y(y_), theta(t_), parent(p) {}
};

class RRTKinematic : public PlanningNode {
public:
    RRTKinematic() : PlanningNode("rrt_kinematic"), following_(false), path_idx_(0) {
        srand(time(nullptr));
    }

    ~RRTKinematic() { clearTree(); }

private:

    static constexpr double V0      = 0.2;    // velocidade linear
    static constexpr double W_MAX   = 0.5;    // limite velocidade angular 
    static constexpr double DT      = 1.0;   
    static constexpr double W_EPS   = 1e-4;   
    static constexpr double GOAL_EPS  = 0.3;
    static constexpr double GOAL_BIAS = 0.10;
    static constexpr int    MAX_ITER  = 10000;
    static constexpr double ANG_TOL = 0.05;
    static constexpr double POS_TOL = 0.15;
    static constexpr double ANG_VEL = 0.80;

    std::vector<TreeNode*> tree_;
    std::vector<TreeNode*> path_;   
    bool following_;
    int  path_idx_;

    double randDouble() { return (double)rand() / RAND_MAX; }

    double wrapAngle(double a) { return atan2(sin(a), cos(a)); }

    bool isFree(double x, double y) {
        if (!map_msg_) return true;
        const auto& info = map_msg_->info;
        int gx = (x - info.origin.position.x) / info.resolution;
        int gy = (y - info.origin.position.y) / info.resolution;
        if (gx < 0 || gy < 0 || gx >= (int)info.width || gy >= (int)info.height)
            return false;
        int val = map_msg_->data[gy * info.width + gx];
        return val >= 0 && val < 50;
    }

    TreeNode* nearest(double x, double y) {
        TreeNode* best = nullptr;
        double dmin = 1e9;
        for (TreeNode* n : tree_) {
            double d = hypot(n->x - x, n->y - y);
            if (d < dmin) { dmin = d; best = n; }
        }
        return best;
    }

    TreeNode* extend(TreeNode* n, double tx, double ty) {
        double desired = atan2(ty - n->y, tx - n->x);
        double w = std::clamp(wrapAngle(desired - n->theta), -W_MAX, W_MAX);

        double nx, ny, nt;
        if (fabs(w) < W_EPS) {
            nx = n->x + V0 * DT * cos(n->theta);
            ny = n->y + V0 * DT * sin(n->theta);
            nt = n->theta;
        } else {
            nx = n->x + (V0/w) * (sin(n->theta + w * DT) - sin(n->theta));
            ny = n->y - (V0/w) * (cos(n->theta + w * DT) - cos(n->theta));
            nt = wrapAngle(n->theta + w * DT);
        }

        const int steps = 10;
        for (int i = 1; i <= steps; ++i) {
            double t = DT * i / (double)steps;
            double px, py;
            if (fabs(w) < W_EPS) {
                px = n->x + V0 * t * cos(n->theta);
                py = n->y + V0 * t * sin(n->theta);
            } else {
                px = n->x + (V0/w) * (sin(n->theta + w * t) - sin(n->theta));
                py = n->y - (V0/w) * (cos(n->theta + w * t) - cos(n->theta));
            }
            if (!isFree(px, py)) return nullptr;
        }

        TreeNode* novo = new TreeNode(nx, ny, nt, n);
        tree_.push_back(novo);
        return novo;
    }

    void backtrack(TreeNode* n) {
        path_.clear();
        for (TreeNode* cur = n; cur; cur = cur->parent)
            path_.push_back(cur);
        std::reverse(path_.begin(), path_.end());
    }

    void clearTree() {
        for (TreeNode* n : tree_) delete n;
        tree_.clear();
    }

    bool solve(double sx, double sy, double stheta, double gx, double gy) {
        if (!map_msg_ || !isFree(sx, sy) || !isFree(gx, gy)) return false;

        clearTree();
        tree_.push_back(new TreeNode(sx, sy, stheta));

        const auto& info = map_msg_->info;
        double map_w = info.width  * info.resolution;
        double map_h = info.height * info.resolution;
        double ox    = info.origin.position.x;
        double oy    = info.origin.position.y;

        for (int i = 0; i < MAX_ITER; i++) {
            double rx = (randDouble() < GOAL_BIAS) ? gx : ox + randDouble() * map_w;
            double ry = (randDouble() < GOAL_BIAS) ? gy : oy + randDouble() * map_h;

            TreeNode* near = nearest(rx, ry);
            if (!near) continue;

            TreeNode* novo = extend(near, rx, ry);
            if (!novo) continue;

            if (hypot(novo->x - gx, novo->y - gy) < GOAL_EPS) {
                backtrack(novo);
                return true;
            }
        }
        return false;
    }

    void draw() {
        visualization_msgs::msg::MarkerArray arr;
        nodes_marker_.points.clear();
        edges_marker_.points.clear();

        for (TreeNode* n : tree_) {
            geometry_msgs::msg::Point p;
            p.x = n->x; p.y = n->y;
            nodes_marker_.points.push_back(p);

            if (n->parent) {
                geometry_msgs::msg::Point a, b;
                a.x = n->parent->x; a.y = n->parent->y;
                b.x = n->x;         b.y = n->y;
                edges_marker_.points.push_back(a);
                edges_marker_.points.push_back(b);
            }
        }
        arr.markers.push_back(nodes_marker_);
        arr.markers.push_back(edges_marker_);

        if (!path_.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp    = now();
            m.ns              = "path";
            m.id              = 0;
            m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            m.scale.x         = 0.08;
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            m.pose.orientation.w = 1.0;
            for (TreeNode* n : path_) {
                geometry_msgs::msg::Point p;
                p.x = n->x; p.y = n->y;
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }
        marker_pub_->publish(arr);
    }

    void on_goal() override {
        if (!has_goal() || !map_msg_) return;
        auto [gx, gy] = goal_;

        following_ = false;
        if (solve(x_, y_, theta_, gx, gy)) {
            path_idx_  = 0;
            following_ = true;
        }
        draw();
    }

    void control_loop() override {
        if (!following_ || path_.empty()) {
            stop();
            return;
        }

        while (path_idx_ < (int)path_.size()) {
            double dx = path_[path_idx_]->x - x_;
            double dy = path_[path_idx_]->y - y_;
            double dist = hypot(dx, dy);
            
            bool reached = (dist < POS_TOL);

            if (!reached && path_idx_ + 1 < (int)path_.size()) {
                double dx2 = path_[path_idx_ + 1]->x - x_;
                double dy2 = path_[path_idx_ + 1]->y - y_;
                if (hypot(dx2, dy2) < dist) {
                    reached = true; 
                }
            }

            if (reached) {
                path_idx_++;
            } else {
                break;
            }
        }

        if (path_idx_ >= (int)path_.size()) {
            stop();
            if (following_) {
                clear_goal();
                following_ = false;
                RCLCPP_INFO(get_logger(), "[rrt_kinematic] Goal reached!");
            }
            draw();
            return;
        }

        TreeNode* wp = path_[path_idx_];
        double dx = wp->x - x_, dy = wp->y - y_;
        double target_angle = atan2(dy, dx);
        double err = wrapAngle(target_angle - theta_);

        double v = V0;

        if (fabs(err) > M_PI / 3.0) {
            v = 0.0;
        } else if (fabs(err) > M_PI / 6.0) {
            v = V0 * 0.4;
        }

        double w = std::clamp(2.5 * err, -ANG_VEL, ANG_VEL);
        
        publish_velocity(v, w);
        draw();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTKinematic>());
    rclcpp::shutdown();
}