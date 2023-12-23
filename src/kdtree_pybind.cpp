#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kdtree.cpp"

PYBIND11_MODULE(kdtree, m) {
    namespace py = pybind11;
    m.doc() = "KD-tree implementation";

    py::class_<Point>(m, "Point")
        .def(py::init<const std::vector<double>&>())
        .def_readonly("coordinates", &Point::coordinates);

    py::class_<KDNode>(m, "KDNode")
        .def(py::init<const Point&>())
        .def_readonly("point", &KDNode::point);

    py::class_<KDTree>(m, "KDTree")
        .def(py::init<>())
        .def("insert", &KDTree::insert)
        .def("nearest_neighbor", &KDTree::nearestNeighbor)
        .def("range_search", &KDTree::rangeSearch)
        .def("traverse_and_print", &KDTree::traverseAndPrint)
        .def("find_min_value_in_dimension", &KDTree::findMinValueInDimension)
        .def("find_max_value_in_dimension", &KDTree::findMaxValueInDimension)
        .def("contains_point", &KDTree::containsPoint);
}
