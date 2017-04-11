R* Tree for C++
===============

In 2008 I created this R* Tree implementation in C++. My goal for this
implementation was as follows:

* Simple API – Most existing implementations I’ve seen are far from simple
* Generic enough to use with most types, with low overhead
* N dimensions for bounding boxes
* Take advantage of modern C++ features to get decent performance

This is a header-only implementation of an R tree with an R* index, and makes
heavy use of templates, STL, and STL-style functors; but it should work in any
relatively modern C++ compiler. To do searching for nodes, this implementation
uses the Visitor pattern via functors — there are examples and documentation in
RStarVisitor.h.

Maintenance
===========

I do not use or update this code anymore, but I welcome pull requests.

License
=======

GNU Lesser Public License v2.1

Author
======

Dustin Spicuzza (dustin@virtualroadside.com)