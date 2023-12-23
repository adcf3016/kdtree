#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kdtree_mod.cpp"

PYBIND11_MODULE(kdtree_mod, m) {
    namespace py = pybind11;
    m.doc() = "KD-tree implementation";

    py::class_<Point>(m, "Point")
        .def(py::init<const std::vector<double>&>())
        .def_readonly("coordinates", &Point::coordinates);

    py::class_<KDNode>(m, "KDNode")
        .def(py::init<const Point&, size_t>())
        .def_readonly("point", &KDNode::point)
        .def_readonly("split_dimension", &KDNode::splitDimension);

    py::class_<KDTree>(m, "KDTree")
        .def(py::init<>())
        .def("insert", &KDTree::insert)
        .def("build", &KDTree::build)
        .def("nearest_neighbor", &KDTree::nearestNeighbor)
        .def("range_search", &KDTree::rangeSearch)
        .def("traverse_and_print", &KDTree::traverseAndPrint)
        .def("find_min_value_in_dimension", &KDTree::findMinValueInDimension)
        .def("find_max_value_in_dimension", &KDTree::findMaxValueInDimension)
        .def("contains_point", &KDTree::containsPoint);
}
