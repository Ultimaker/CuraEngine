window.BENCHMARK_DATA = {
  "lastUpdate": 1668598308193,
  "repoUrl": "https://github.com/Ultimaker/CuraEngine",
  "entries": {
    "C++ Benchmark": [
      {
        "commit": {
          "author": {
            "email": "j.spijker@ultimaker.com",
            "name": "jspijker",
            "username": "jellespijker"
          },
          "committer": {
            "email": "j.spijker@ultimaker.com",
            "name": "jspijker",
            "username": "jellespijker"
          },
          "distinct": true,
          "id": "dc63b191e30c13e09a9c2f883ad3b68dbc98747c",
          "message": "Define needs",
          "timestamp": "2022-11-16T12:07:29+01:00",
          "tree_id": "0edb4f31b13c0ea7322d9cac4891e42ae89c830b",
          "url": "https://github.com/Ultimaker/CuraEngine/commit/dc63b191e30c13e09a9c2f883ad3b68dbc98747c"
        },
        "date": 1668598301181,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/3",
            "value": 159440.21054512958,
            "unit": "ns/iter",
            "extra": "iterations: 5595\ncpu: 159295.49597855227 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/5",
            "value": 542142.7190000259,
            "unit": "ns/iter",
            "extra": "iterations: 1000\ncpu: 542039.6999999998 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/999999",
            "value": 5578733.950000014,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 5578052 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}