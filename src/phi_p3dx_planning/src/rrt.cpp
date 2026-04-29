#include "planning_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct TreeNode {
    double x, y;
    TreeNode* parent;
    TreeNode(double x_, double y_, TreeNode* p = nullptr)
        : x(x_), y(y_), parent(p) {}
};

class RRT : public PlanningNode {
public:
    RRT() : PlanningNode("rrt"), following_(false), path_idx_(0) {
        srand(time(nullptr));
    }

    ~RRT() { clearTree(); }

private:

    static constexpr double STEP      = 0.5;   // tamanho do passo
    static constexpr double GOAL_EPS  = 0.3;   // tolerância do objetivo
    static constexpr double GOAL_BIAS = 0.10;  // prob. de amostrar o objetivo
    static constexpr int    MAX_ITER  = 10000; // limite de iterações
    static constexpr double ANG_TOL   = 0.05;  // tolerância angular
    static constexpr double POS_TOL   = 0.15;  // tolerância de posição 
    static constexpr double LIN_VEL   = 0.25;  // velocidade linear 
    static constexpr double ANG_VEL   = 0.80;  // limite velocidade angular

    std::vector<TreeNode*> tree_;
    std::vector<std::pair<double,double>> path_;
    bool following_;
    int  path_idx_;

    double randDouble() { 
        return (double)rand() / RAND_MAX; 
    }

    double wrapAngle(double a) { 
        return atan2(sin(a), cos(a)); 
    }

    bool isFree(double x, double y, double margin = 0.0) {
        if (!map_msg_) 
            return true;
        const auto& info = map_msg_->info;
        int gx = (x - info.origin.position.x) / info.resolution;
        int gy = (y - info.origin.position.y) / info.resolution;

        if (margin <= 0.0) {
            if (gx < 0 || gy < 0 || gx >= (int)info.width || gy >= (int)info.height)
                return false;
            int val = map_msg_->data[gy * info.width + gx];
            return val >= 0 && val < 50;  
        }

        int r_cells = std::ceil(margin / info.resolution);
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
            for (int dx = -r_cells; dx <= r_cells; ++dx) {
                if (dx*dx + dy*dy <= r_cells*r_cells) {
                    int nx = gx + dx;
                    int ny = gy + dy;
                    if (nx < 0 || ny < 0 || nx >= (int)info.width || ny >= (int)info.height)
                        return false;
                    int val = map_msg_->data[ny * info.width + nx];
                    if (val < 0 || val >= 50)
                        return false;
                }
            }
        }
        return true;
    }



    // Retorna o nó mais próximo de (x, y)
    TreeNode* nearest(double x, double y) {
        TreeNode* best = nullptr;
        double dmin = 1e9;
        for (TreeNode* n : tree_) {
            double d = hypot(n->x - x, n->y - y);
            if (d < dmin) { 
                dmin = d; 
                best = n; 
            }
        }
        return best;
    }

    // Expande a árvore e retorna novo nó ou nullptr
    TreeNode* extend(TreeNode* n, double tx, double ty) {
        double dx = tx - n->x, dy = ty - n->y;
        double d  = hypot(dx, dy);
        if (d > STEP) { 
            dx = dx/d * STEP; 
            dy = dy/d * STEP; 
        }

        const int steps = 10;
        double last_x = n->x;
        double last_y = n->y;
        bool valid = false;

        for (int i = 1; i <= steps; ++i) {
            double t = i / (double)steps;
            double px = n->x + t * dx;
            double py = n->y + t * dy;
            if (!isFree(px, py, 0.25)) 
                break;
            last_x = px;
            last_y = py;
            valid = true;
        }

        if (!valid) 
            return nullptr;

        TreeNode* novo = new TreeNode(last_x, last_y, n);
        tree_.push_back(novo);
        return novo;
    }

    // Reconstrói o caminho do nó até a raiz
    void backtrack(TreeNode* n) {
        path_.clear();
        for (TreeNode* cur = n; cur; cur = cur->parent)
            path_.emplace_back(cur->x, cur->y);
        std::reverse(path_.begin(), path_.end());
    }

    void clearTree() {
        for (TreeNode* n : tree_) 
            delete n;
        tree_.clear();
    }

    bool solve(double sx, double sy, double gx, double gy) {
        if (!map_msg_ || !isFree(sx, sy) || !isFree(gx, gy)) 
            return false;

        clearTree();
        tree_.push_back(new TreeNode(sx, sy));

        const auto& info = map_msg_->info;
        double map_w = info.width  * info.resolution;
        double map_h = info.height * info.resolution;
        double ox = info.origin.position.x;
        double oy = info.origin.position.y;

        for (int i = 0; i < MAX_ITER; i++) {
            double rx = (randDouble() < GOAL_BIAS) ? gx : ox + randDouble() * map_w;
            double ry = (randDouble() < GOAL_BIAS) ? gy : oy + randDouble() * map_h;

            TreeNode* near = nearest(rx, ry);
            if (!near) 
                continue;

            TreeNode* novo = extend(near, rx, ry);
            if (!novo) 
                continue;

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
                a.x = n->parent->x; 
                a.y = n->parent->y;
                b.x = n->x;         
                b.y = n->y;
                edges_marker_.points.push_back(a);
                edges_marker_.points.push_back(b);
            }
        }
        arr.markers.push_back(nodes_marker_);
        arr.markers.push_back(edges_marker_);

        if (!path_.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = now();
            m.ns = "path";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.scale.x = 0.08;
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            m.pose.orientation.w = 1.0;
            for (auto& [px, py] : path_) {
                geometry_msgs::msg::Point p;
                p.x = px; p.y = py;
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }
        marker_pub_->publish(arr);
    }

    void on_goal() override {
        if (!has_goal() || !map_msg_) 
            return;
        auto [gx, gy] = goal_;

        following_ = false;
        if (solve(x_, y_, gx, gy)) {
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
            auto [tx, ty] = path_[path_idx_];
            double dx = tx - x_, dy = ty - y_;
            double dist = hypot(dx, dy);
            
            bool reached = (dist < POS_TOL);

            if (!reached && path_idx_ + 1 < (int)path_.size()) {
                auto [tx2, ty2] = path_[path_idx_ + 1];
                if (hypot(tx2 - x_, ty2 - y_) < dist) {
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
            }
            draw();
            return;
        }

        auto [tx, ty] = path_[path_idx_];
        double dx = tx - x_, dy = ty - y_;
        double target_angle = atan2(dy, dx);
        double err = wrapAngle(target_angle - theta_);

        double v = LIN_VEL;
  
        if (fabs(err) > M_PI / 3.0) {
            v = 0.0; 
        } else if (fabs(err) > M_PI / 6.0) {
            v = LIN_VEL * 0.4;
        }

        double w = std::clamp(2.5 * err, -ANG_VEL, ANG_VEL);
        
        publish_velocity(v, w);
        draw();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT>());
    rclcpp::shutdown();
}