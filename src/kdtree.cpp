#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
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
    KDTree() : k(0), root(nullptr) {}

    void insert(const Point& point) {
        if (k == 0) {
            // If the dimension is not set, set it based on the length of the point
            assert(point.coordinates.size() > 0 && "Point must have at least one coordinate.");
            k = point.coordinates.size();
        }

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

    void traverseAndPrint() const {
        std::cout << "KD-tree traversal:" << std::endl;
        traverseAndPrintRecursive(root, 0);
        std::cout << std::endl;
    }

    double findMinValueInDimension(size_t dimension) const {
        assert(dimension < k && "Invalid dimension");
        return minInDimensionRecursive(root, dimension, 0);
    }

    double findMaxValueInDimension(size_t dimension) const {
        assert(dimension < k && "Invalid dimension");
        return maxInDimensionRecursive(root, dimension, 0);
    }

    bool containsPoint(const Point& point) const {
        return containsPointRecursive(root, point, 0);
    }

   private:
    size_t k;
    KDNode* root;

    KDNode* insertRecursive(KDNode* node, const Point& point, size_t depth) {
        if (node == nullptr) {
            return new KDNode(point);
        }

        if (depth % k == 0) {
            // If depth % k is 0, it means we need to split along the x-axis,
            // so we choose the dimension based on the current depth.
            size_t currentDimension = depth / k % point.coordinates.size();

            if (point.coordinates[currentDimension] < node->point.coordinates[currentDimension]) {
                node->left = insertRecursive(node->left, point, depth + 1);
            } else {
                node->right = insertRecursive(node->right, point, depth + 1);
            }
        } else {
            // If depth % k is not 0, it means we need to split along a different axis,
            // so we choose the dimension based on depth % k.
            size_t currentDimension = depth % k;

            if (point.coordinates[currentDimension] < node->point.coordinates[currentDimension]) {
                node->left = insertRecursive(node->left, point, depth + 1);
            } else {
                node->right = insertRecursive(node->right, point, depth + 1);
            }
        }

        return node;
    }

    KDNode nearestNeighborRecursive(KDNode* node, const Point& target, size_t depth) const {
        if (node == nullptr) {
            return KDNode(Point(std::vector<double>(k, 0.0)));
        }

        if (depth % k == 0) {
            // If depth % k is 0, it means we need to split along the x-axis,
            // so we choose the dimension based on the current depth.
            size_t currentDimension = depth / k % target.coordinates.size();

            KDNode* nextBranch = (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) ? node->left : node->right;
            KDNode* oppositeBranch = (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) ? node->right : node->left;

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
        } else {
            // If depth % k is not 0, it means we need to split along a different axis,
            // so we choose the dimension based on depth % k.
            size_t currentDimension = depth % k;

            KDNode* nextBranch = (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) ? node->left : node->right;
            KDNode* oppositeBranch = (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) ? node->right : node->left;

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
    }

    void rangeSearchRecursive(KDNode* node, const Point& lowerBound, const Point& upperBound, size_t depth, std::vector<Point>& result) const {
        if (node == nullptr) {
            return;
        }

        if (depth % k == 0) {
            // If depth % k is 0, it means we need to split along the x-axis,
            // so we choose the dimension based on the current depth.
            size_t currentDimension = depth / k % lowerBound.coordinates.size();

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
        } else {
            // If depth % k is not 0, it means we need to split along a different axis,
            // so we choose the dimension based on depth % k.
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
    }

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

    double minInDimensionRecursive(KDNode* node, size_t dimension, size_t depth) const {
        if (node == nullptr) {
            // Return a large value for nodes that don't exist
            return std::numeric_limits<double>::infinity();
        }

        size_t currentDimension = depth % k;

        if (dimension == currentDimension) {
            // If this is the target dimension, recurse only on the relevant branch
            if (node->left == nullptr) {
                return node->point.coordinates[dimension];
            } else {
                return minInDimensionRecursive(node->left, dimension, depth + 1);
            }
        } else {
            // If this is not the target dimension, consider both branches
            double leftMin = minInDimensionRecursive(node->left, dimension, depth + 1);
            double rightMin = minInDimensionRecursive(node->right, dimension, depth + 1);
            return std::min(node->point.coordinates[dimension], std::min(leftMin, rightMin));
        }
    }

    double maxInDimensionRecursive(KDNode* node, size_t dimension, size_t depth) const {
        if (node == nullptr) {
            // Return a small value for nodes that don't exist
            return -std::numeric_limits<double>::infinity();
        }

        size_t currentDimension = depth % k;

        if (dimension == currentDimension) {
            // If this is the target dimension, recurse only on the relevant branch
            if (node->right == nullptr) {
                return node->point.coordinates[dimension];
            } else {
                return maxInDimensionRecursive(node->right, dimension, depth + 1);
            }
        } else {
            // If this is not the target dimension, consider both branches
            double leftMax = maxInDimensionRecursive(node->left, dimension, depth + 1);
            double rightMax = maxInDimensionRecursive(node->right, dimension, depth + 1);
            return std::max(node->point.coordinates[dimension], std::max(leftMax, rightMax));
        }
    }

    bool containsPointRecursive(KDNode* node, const Point& target, size_t depth) const {
        if (node == nullptr) {
            return false;
        }

        size_t currentDimension = depth % k;

        if (node->point.coordinates == target.coordinates) {
            return true;
        }

        if (target.coordinates[currentDimension] < node->point.coordinates[currentDimension]) {
            return containsPointRecursive(node->left, target, depth + 1);
        } else {
            return containsPointRecursive(node->right, target, depth + 1);
        }
    }
};

int main() {
    // Example usage
    KDTree kdTree;

    for (const auto& point : {Point({1.0, 2.0, 3.0}),
                              Point({4.0, 5.0, 6.0}),
                              Point({7.0, 8.0, 9.0}),
                              Point({10.0, 11.0, 12.0}),
                              Point({13.0, 14.0, 15.0}),
                              Point({16.0, 17.0, 18.0}),
                              Point({19.0, 20.0, 21.0}),
                              Point({22.0, 23.0, 24.0}),
                              Point({25.0, 26.0, 27.0}),
                              Point({28.0, 29.0, 30.0}),
                              Point({2.0, 3.0, 4.0}),
                              Point({5.0, 6.0, 7.0}),
                              Point({8.0, 9.0, 10.0}),
                              Point({2.6, 5.0, 120.0}),
                              Point({1.0, 3.0, 6.0}),
                              Point({4.0, 6.0, 3.0}),
                              Point({5.0, 4.0, 8.0})}) {
        kdTree.insert(point);
    }

    // kdTree.insert(Point({2.0, 3.0, 4.0}));
    // kdTree.insert(Point({5.0, 6.0, 7.0}));
    // kdTree.insert(Point({8.0, 9.0, 10.0}));
    // kdTree.insert(Point({2.6, 5.0, 120.0}));
    // kdTree.insert(Point({1.0, 3.0, 6.0}));
    // kdTree.insert(Point({4.0, 6.0, 3.0}));
    // kdTree.insert(Point({5.0, 4.0, 8.0}));
    // kdTree.insert(Point({1.0}));
    // kdTree.insert(Point({2.0}));
    // kdTree.insert(Point({3.0}));
    // kdTree.insert(Point({4.0}));
    // kdTree.insert(Point({5.0}));
    // kdTree.insert(Point({6.0}));
    // kdTree.insert(Point({7.0}));
    // kdTree.insert(Point({8.0}));
    // kdTree.insert(Point({9.0}));
    // kdTree.insert(Point({10.0}));
    // kdTree.insert(Point({11.0}));
    // kdTree.insert(Point({12.0}));

    Point lowerBound({2.0, 3.0, 3.0});
    Point upperBound({6.0, 7.0, 8.0});

    // Nearest neighbor search
    Point target({3.0, 5.0, 8.0});
    Point nearestNeighbor = kdTree.nearestNeighbor(target);
    std::cout << "Nearest neighbor: ";
    for (double coord : nearestNeighbor.coordinates) {
        std::cout << coord << " ";
    }
    std::cout << std::endl;

    // Range search
    std::vector<Point> result = kdTree.rangeSearch(lowerBound, upperBound);
    std::cout << "Points within the specified range: ";
    for (const Point& point : result) {
        for (double coord : point.coordinates) {
            std::cout << coord << " ";
        }
        std::cout << "| ";
    }
    std::cout << std::endl;

    size_t dimensionToCheck = 0;
    double minValue = kdTree.findMinValueInDimension(dimensionToCheck);
    double maxValue = kdTree.findMaxValueInDimension(dimensionToCheck);

    std::cout << "Min value in dimension " << dimensionToCheck << ": " << minValue << std::endl;
    std::cout << "Max value in dimension " << dimensionToCheck << ": " << maxValue << std::endl;

    Point pointToCheck({16.1, 17.0, 18.0});
    bool containsPoint = kdTree.containsPoint(pointToCheck);
    std::cout << "Contains point: " << (containsPoint ? "Yes" : "No") << std::endl;

    kdTree.traverseAndPrint();

    return 0;
}
