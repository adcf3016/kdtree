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
    size_t splitDimension;  // New member to store the split dimension
    KDNode* left;
    KDNode* right;

    KDNode(const Point& p, size_t dim) : point(p), splitDimension(dim), left(nullptr), right(nullptr) {}
};

class KDTree {
   public:
    KDTree() : k(0), root(nullptr) {}

    void insert(const Point& point) {
        if (k == 0) {
            assert(point.coordinates.size() > 0 && "Point must have at least one coordinate.");
            k = point.coordinates.size();
        }

        points.push_back(point);
    }

    void build() {
        root = buildRecursive(points, 0, points.size(), 0);
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
    std::vector<Point> points;

    KDNode* buildRecursive(std::vector<Point>& points, size_t begin, size_t end, size_t depth) {
        if (begin >= end) {
            return nullptr;
        }

        size_t currentDimension = findBestSplitDimension(points, begin, end);
        size_t medianIndex = findMedianIndex(points, begin, end, currentDimension);

        KDNode* node = new KDNode(points[medianIndex], currentDimension);

        node->left = buildRecursive(points, begin, medianIndex, depth + 1);
        node->right = buildRecursive(points, medianIndex + 1, end, depth + 1);

        return node;
    }

    size_t findBestSplitDimension(std::vector<Point>& points, size_t begin, size_t end) const {
        double bestVariance = -1.0;
        size_t bestDimension = 0;

        for (size_t dim = 0; dim < k; ++dim) {
            double variance = calculateVariance(points, begin, end, dim);
            if (variance > bestVariance) {
                bestVariance = variance;
                bestDimension = dim;
            }
        }

        return bestDimension;
    }

    double calculateVariance(std::vector<Point>& points, size_t begin, size_t end, size_t dimension) const {
        double sum = 0.0;
        double sumSquared = 0.0;

        for (size_t i = begin; i < end; ++i) {
            double value = points[i].coordinates[dimension];
            sum += value;
            sumSquared += value * value;
        }

        double mean = sum / static_cast<double>(end - begin);
        double variance = (sumSquared / static_cast<double>(end - begin)) - (mean * mean);

        return variance;
    }

    size_t findMedianIndex(std::vector<Point>& points, size_t begin, size_t end, size_t currentDimension) {
        size_t medianIndex = begin + (end - begin) / 2;

        std::sort(points.begin() + begin, points.begin() + end,
                  [currentDimension](const Point& a, const Point& b) {
                      return a.coordinates[currentDimension] < b.coordinates[currentDimension];
                  });

        return medianIndex;
    }

    KDNode nearestNeighborRecursive(KDNode* node, const Point& target, size_t depth) const {
        if (node == nullptr) {
            return KDNode(Point(std::vector<double>(k, 0.0)), 0);
        }

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

    void traverseAndPrintRecursive(KDNode* node, size_t depth) const {
        if (node != nullptr) {
            traverseAndPrintRecursive(node->left, depth + 1);

            std::cout << "Depth " << depth << ": ";
            for (size_t i = 0; i < k; ++i) {
                std::cout << node->point.coordinates[i] << " ";
            }
            std::cout << "(Split Dim: " << node->splitDimension << ")" << std::endl;

            traverseAndPrintRecursive(node->right, depth + 1);
        }
    }

    double minInDimensionRecursive(KDNode* node, size_t dimension, size_t depth) const {
        if (node == nullptr) {
            // Return a large value for nodes that don't exist
            return std::numeric_limits<double>::infinity();
        }

        size_t currentDimension = node->splitDimension;

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

        size_t currentDimension = node->splitDimension;

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

        size_t currentDimension = node->splitDimension;

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
    KDTree kdTree;

    for (const auto& point : {
             Point({1.0, 2.0, 3.0}),
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

    kdTree.build();
    Point target({3.0, 5.0, 8.0});
    Point nearestNeighbor = kdTree.nearestNeighbor(target);
    std::cout << "Nearest neighbor: ";
    for (double coord : nearestNeighbor.coordinates) {
        std::cout << coord << " ";
    }
    std::cout << std::endl;

    size_t dimensionToCheck = 2;
    double minValue = kdTree.findMinValueInDimension(dimensionToCheck);
    double maxValue = kdTree.findMaxValueInDimension(dimensionToCheck);

    std::cout << "Min value in dimension " << dimensionToCheck << ": " << minValue << std::endl;
    std::cout << "Max value in dimension " << dimensionToCheck << ": " << maxValue << std::endl;

    Point pointToCheck({16.0, 17.0, 18.0});
    bool containsPoint = kdTree.containsPoint(pointToCheck);
    std::cout << "Contains point: " << (containsPoint ? "Yes" : "No") << std::endl;

    kdTree.traverseAndPrint();

    return 0;
}
