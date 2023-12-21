#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

struct Point {
    std::vector<double> coordinates;

    Point(const std::vector<double>& coords) : coordinates(coords) {}

    double squaredDistance(const Point& other) const {
        double distance = 0;
        for (size_t i = 0; i < coordinates.size(); ++i) {
            double diff = coordinates[i] - other.coordinates[i];
            distance += diff * diff;
        }
        return distance;
    }
};

struct KDNode {
    Point point;
    KDNode* left;
    KDNode* right;

    KDNode(const Point& p) : point(p), left(nullptr), right(nullptr) {}
};

class KDTree {
   public:
    KDTree(size_t dimensions) : k(dimensions), root(nullptr) {}

    void insert(const Point& point) {
        root = insertRecursive(root, point, 0);
    }

    Point nearestNeighbor(const Point& target) const {
        return nearestNeighborRecursive(root, target, 0).point;
    }

    std::vector<Point> rangeSearch(const Point& lowerBound, const Point& upperBound) const {
        std::vector<Point> result;
        rangeSearchRecursive(root, lowerBound, upperBound, 0, result);
        return result;
    }

    // Tree traversal and print function
    void traverseAndPrint() const {
        std::cout << "KD-tree traversal:" << std::endl;
        traverseAndPrintRecursive(root, 0);
        std::cout << std::endl;
    }

   private:
    size_t k;
    KDNode* root;

    KDNode* insertRecursive(KDNode* node, const Point& point, size_t depth) {
        if (node == nullptr) {
            return new KDNode(point);
        }

        size_t currentDimension = depth % k;

        if (point.coordinates[currentDimension] < node->point.coordinates[currentDimension]) {
            node->left = insertRecursive(node->left, point, depth + 1);
        } else {
            node->right = insertRecursive(node->right, point, depth + 1);
        }

        return node;
    }

    KDNode nearestNeighborRecursive(KDNode* node, const Point& target, size_t depth) const {
        if (node == nullptr) {
            return KDNode(Point(std::vector<double>(k, 0.0)));
        }

        size_t currentDimension = depth % k;

        KDNode* nextBranch = nullptr;
        KDNode* oppositeBranch = nullptr;

        if (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) {
            nextBranch = node->left;
            oppositeBranch = node->right;
        } else {
            nextBranch = node->right;
            oppositeBranch = node->left;
        }

        KDNode best = nearestNeighborRecursive(nextBranch, target, depth + 1);

        if (target.squaredDistance(best.point) > std::abs(target.coordinates[currentDimension] - node->point.coordinates[currentDimension])) {
            KDNode opposite = nearestNeighborRecursive(oppositeBranch, target, depth + 1);
            if (target.squaredDistance(opposite.point) < target.squaredDistance(best.point)) {
                best = opposite;
            }
        }

        if (target.squaredDistance(node->point) < target.squaredDistance(best.point)) {
            best = *node;
        }

        return best;
    }

    void rangeSearchRecursive(KDNode* node, const Point& lowerBound, const Point& upperBound, size_t depth, std::vector<Point>& result) const {
        if (node == nullptr) {
            return;
        }

        size_t currentDimension = depth % k;

        if (node->point.coordinates[currentDimension] >= lowerBound.coordinates[currentDimension] &&
            node->point.coordinates[currentDimension] <= upperBound.coordinates[currentDimension]) {
            bool inRange = true;
            for (size_t i = 0; i < k; ++i) {
                if (node->point.coordinates[i] < lowerBound.coordinates[i] || node->point.coordinates[i] > upperBound.coordinates[i]) {
                    inRange = false;
                    break;
                }
            }

            if (inRange) {
                result.push_back(node->point);
            }
        }

        rangeSearchRecursive(node->left, lowerBound, upperBound, depth + 1, result);
        rangeSearchRecursive(node->right, lowerBound, upperBound, depth + 1, result);
    }

    // Helper function for tree traversal and print
    void traverseAndPrintRecursive(KDNode* node, size_t depth) const {
        if (node != nullptr) {
            traverseAndPrintRecursive(node->left, depth + 1);

            std::cout << "Depth " << depth << ": ";
            for (double coord : node->point.coordinates) {
                std::cout << coord << " ";
            }
            std::cout << std::endl;

            traverseAndPrintRecursive(node->right, depth + 1);
        }
    }
};

int main() {
    // Example usage
    // size_t dimensions = 3;
    size_t dimensions = 1;
    KDTree kdTree(dimensions);

    // kdTree.insert(Point({2.0, 3.0, 4.0}));
    // kdTree.insert(Point({5.0, 6.0, 7.0}));
    // kdTree.insert(Point({8.0, 9.0, 10.0}));
    // kdTree.insert(Point({2.6, 5.0, 120.0}));
    // kdTree.insert(Point({1.0, 3.0, 6.0}));
    // kdTree.insert(Point({4.0, 6.0, 3.0}));
    // kdTree.insert(Point({5.0, 4.0, 8.0}));
    kdTree.insert(Point({1.0}));
    kdTree.insert(Point({2.0}));
    kdTree.insert(Point({3.0}));
    kdTree.insert(Point({4.0}));
    kdTree.insert(Point({5.0}));
    kdTree.insert(Point({6.0}));
    kdTree.insert(Point({7.0}));
    kdTree.insert(Point({8.0}));
    kdTree.insert(Point({9.0}));
    kdTree.insert(Point({10.0}));
    kdTree.insert(Point({11.0}));
    kdTree.insert(Point({12.0}));

    kdTree.traverseAndPrint();

    return 0;
}
