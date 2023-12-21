#include <algorithm>
#include <cassert>
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

   private:
    size_t k;
    KDNode* root;
    std::vector<Point> points;

    KDNode* buildRecursive(std::vector<Point>& points, size_t begin, size_t end, size_t depth) {
        if (begin >= end) {
            return nullptr;
        }

        size_t currentDimension = depth % k;
        size_t medianIndex = findMedianIndex(points, begin, end, currentDimension);

        KDNode* node = new KDNode(points[medianIndex], currentDimension);

        node->left = buildRecursive(points, begin, medianIndex, depth + 1);
        node->right = buildRecursive(points, medianIndex + 1, end, depth + 1);

        return node;
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
             Point({28.0, 29.0, 30.0})
             //  Point({1.0}),
             //  Point({2.0}),
             //  Point({3.0}),
             //  Point({4.0}),
             //  Point({5.0}),
             //  Point({6.0}),
             //  Point({7.0}),
             //  Point({8.0}),
             //  Point({9.0}),
             //  Point({10.0}),
             //  Point({11.0}),
             //  Point({12.0})
         }) {
        kdTree.insert(point);
    }

    kdTree.build();
    kdTree.traverseAndPrint();

    return 0;
}
