Pipeline
========

This document serves as an overview of how the slicing process works from front to back.

There are 5 main stages involved in slicing an object. Not all of them are executed strictly in sequence though. To reduce memory overhead and improve multi-threaded performance, CuraEngine executes some of these steps in a producer-consumer pattern: One thread produces data in an earlier stage that is consumed by another thread computing a later stage at the same time.

Slicing
-------

The first step in the pipeline is the slicing stage. This converts a 3D mesh into 2D layers. First it determines the heights at which to produce cross sections. Then it creates cross sections of every triangle at those heights, which will normally be a line. Then it stitches these lines together to form polygons wherever possible.