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

        size_t currentDimension = findMaxVarianceDimension(node, depth);

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

    size_t findMaxVarianceDimension(KDNode* node, size_t depth) const {
        size_t dimensions = node->point.coordinates.size();
        size_t currentDimension = depth % dimensions;

        double maxVariance = 0.0;
        size_t maxVarianceDimension = currentDimension;

        for (size_t i = 0; i < dimensions; ++i) {
            double variance = calculateVariance(node, i);
            if (variance > maxVariance) {
                maxVariance = variance;
                maxVarianceDimension = i;
            }
        }

        return maxVarianceDimension;
    }

    double calculateVariance(KDNode* node, size_t dimension) const {
        size_t count = 0;
        double mean = 0.0;
        double m2 = 0.0;

        calculateVarianceRecursive(node, dimension, count, mean, m2);

        return m2 / count;
    }

    void calculateVarianceRecursive(KDNode* node, size_t dimension, size_t& count, double& mean, double& m2) const {
        if (node != nullptr) {
            count++;
            double delta = node->point.coordinates[dimension] - mean;
            mean += delta / count;
            double delta2 = node->point.coordinates[dimension] - mean;
            m2 += delta * delta2;

            calculateVarianceRecursive(node->left, dimension, count, mean, m2);
            calculateVarianceRecursive(node->right, dimension, count, mean, m2);
        }
    }

    Point findMedianPoint(KDNode* node, size_t dimension) const {
        std::vector<double> values;

        collectValuesRecursive(node, dimension, values);

        std::sort(values.begin(), values.end());

        size_t medianIndex = values.size() / 2;

        return Point({values[medianIndex]});
    }

    void collectValuesRecursive(KDNode* node, size_t dimension, std::vector<double>& values) const {
        if (node != nullptr) {
            values.push_back(node->point.coordinates[dimension]);

            collectValuesRecursive(node->left, dimension, values);
            collectValuesRecursive(node->right, dimension, values);
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
};

int main() {
    // Example usage
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

    // Point lowerBound({2.0, 3.0, 3.0});
    // Point upperBound({6.0, 7.0, 8.0});
    Point lowerBound({2.0});
    Point upperBound({7.0});

    // Nearest neighbor search
    // Point target({3.0, 5.0, 8.0});
    Point target({4.9});
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

    kdTree.traverseAndPrint();

    return 0;
}
