window.BENCHMARK_DATA = {
  "lastUpdate": 1668612248659,
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
      },
      {
        "commit": {
          "author": {
            "email": "j.spijker@ultimaker.com",
            "name": "Jelle Spijker",
            "username": "jellespijker"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1feac68da04e58ab76198f752540f25342165a21",
          "message": "don't use external caclche path",
          "timestamp": "2022-11-16T16:16:51+01:00",
          "tree_id": "5961e7d5ac88d24efe162c6046bc412d8155be18",
          "url": "https://github.com/Ultimaker/CuraEngine/commit/1feac68da04e58ab76198f752540f25342165a21"
        },
        "date": 1668612242720,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "WallsComputationTest/generateWalls/3",
            "value": 5915945.549999151,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 5808414 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/15",
            "value": 18620155.755555365,
            "unit": "ns/iter",
            "extra": "iterations: 45\ncpu: 18617137.777777776 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/999999",
            "value": 44760454.333331585,
            "unit": "ns/iter",
            "extra": "iterations: 18\ncpu: 44753544.44444444 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/3",
            "value": 123294.75310788219,
            "unit": "ns/iter",
            "extra": "iterations: 7481\ncpu: 123252.66675578126 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/15",
            "value": 1650976.565836328,
            "unit": "ns/iter",
            "extra": "iterations: 562\ncpu: 1650800.1779359423 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/999999",
            "value": 6072554.820000278,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 6071340.999999996 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/3",
            "value": 62809.501793548414,
            "unit": "ns/iter",
            "extra": "iterations: 17563\ncpu: 62804.093833627514 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/15",
            "value": 515626.8472539905,
            "unit": "ns/iter",
            "extra": "iterations: 1748\ncpu: 513321.2814645309 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/999999",
            "value": 1592861.6123712403,
            "unit": "ns/iter",
            "extra": "iterations: 485\ncpu: 1592581.8556700996 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}