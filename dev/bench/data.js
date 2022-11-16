window.BENCHMARK_DATA = {
  "lastUpdate": 1668626023469,
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
      },
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
          "id": "32e120dae5aeba69af799e1bf0bcb69e96510c01",
          "message": "Add a simple infill test",
          "timestamp": "2022-11-16T16:57:29+01:00",
          "tree_id": "8bf270ebf4865dd446d86c1be83a0c8174727cee",
          "url": "https://github.com/Ultimaker/CuraEngine/commit/32e120dae5aeba69af799e1bf0bcb69e96510c01"
        },
        "date": 1668616199415,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "WallsComputationTest/generateWalls/3",
            "value": 5729500.910001662,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 5728827 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/15",
            "value": 17375829.24489307,
            "unit": "ns/iter",
            "extra": "iterations: 49\ncpu: 17374814.285714287 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/999999",
            "value": 40632809.99999781,
            "unit": "ns/iter",
            "extra": "iterations: 19\ncpu: 40630231.57894739 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/3",
            "value": 79841.65998610343,
            "unit": "ns/iter",
            "extra": "iterations: 11526\ncpu: 79467.08311643245 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/15",
            "value": 1038852.7746634266,
            "unit": "ns/iter",
            "extra": "iterations: 892\ncpu: 1038765.2466367703 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/999999",
            "value": 4789675.972377918,
            "unit": "ns/iter",
            "extra": "iterations: 181\ncpu: 4789253.59116022 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/3",
            "value": 47558.45655575837,
            "unit": "ns/iter",
            "extra": "iterations: 23651\ncpu: 47554.75032768166 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/15",
            "value": 392089.7349821451,
            "unit": "ns/iter",
            "extra": "iterations: 2264\ncpu: 392055.9628975267 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/999999",
            "value": 1276363.2838502475,
            "unit": "ns/iter",
            "extra": "iterations: 613\ncpu: 1276273.5725938012 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/400",
            "value": 2660578.155556568,
            "unit": "ns/iter",
            "extra": "iterations: 315\ncpu: 2660350.4761904757 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/800",
            "value": 13166729.216981418,
            "unit": "ns/iter",
            "extra": "iterations: 106\ncpu: 13165641.509433959 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/1200",
            "value": 53681116.8999975,
            "unit": "ns/iter",
            "extra": "iterations: 10\ncpu: 53675220.000000045 ns\nthreads: 1"
          }
        ]
      },
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
          "id": "269ae41cc08b7959f7d236103e78d3cbb0d26328",
          "message": "Run benchmarks when a Release tag is created",
          "timestamp": "2022-11-16T17:09:02+01:00",
          "tree_id": "e68459067d93acc162d8ec4f5e2f15dbf2eb6e06",
          "url": "https://github.com/Ultimaker/CuraEngine/commit/269ae41cc08b7959f7d236103e78d3cbb0d26328"
        },
        "date": 1668617387025,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "WallsComputationTest/generateWalls/3",
            "value": 5724430.830000528,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 5722221.000000001 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/15",
            "value": 17324202.959190764,
            "unit": "ns/iter",
            "extra": "iterations: 49\ncpu: 17322659.18367347 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/generateWalls/999999",
            "value": 40692899.57894733,
            "unit": "ns/iter",
            "extra": "iterations: 19\ncpu: 40689057.89473684 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/3",
            "value": 78805.42459496767,
            "unit": "ns/iter",
            "extra": "iterations: 11604\ncpu: 78798.7504308859 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/15",
            "value": 1032100.0258428602,
            "unit": "ns/iter",
            "extra": "iterations: 890\ncpu: 1032028.4269662927 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getRegionOrder/999999",
            "value": 4775268.696132056,
            "unit": "ns/iter",
            "extra": "iterations: 181\ncpu: 4774966.85082873 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/3",
            "value": 48001.30801777522,
            "unit": "ns/iter",
            "extra": "iterations: 23398\ncpu: 47997.7476707411 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/15",
            "value": 393872.66547564225,
            "unit": "ns/iter",
            "extra": "iterations: 2239\ncpu: 393851.36221527506 ns\nthreads: 1"
          },
          {
            "name": "WallsComputationTest/InsetOrderOptimizer_getInsetOrder/999999",
            "value": 1371088.3344483105,
            "unit": "ns/iter",
            "extra": "iterations: 598\ncpu: 1370973.578595318 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/400",
            "value": 2652485.5365081863,
            "unit": "ns/iter",
            "extra": "iterations: 315\ncpu: 2652076.190476192 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/800",
            "value": 13159282.23584706,
            "unit": "ns/iter",
            "extra": "iterations: 106\ncpu: 13158417.924528308 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generateTest/1200",
            "value": 52423129.099997826,
            "unit": "ns/iter",
            "extra": "iterations: 10\ncpu: 52416559.999999985 ns\nthreads: 1"
          }
        ]
      },
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
          "id": "224e45c24ab29c2e1896488d3160c0a23c6dd97c",
          "message": "Add the benchmark workflow",
          "timestamp": "2022-11-16T19:58:51+01:00",
          "tree_id": "cba7c8bf57888cd68fe2d15a18c3b82276c6bcaa",
          "url": "https://github.com/Ultimaker/CuraEngine/commit/224e45c24ab29c2e1896488d3160c0a23c6dd97c"
        },
        "date": 1668626017346,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "InfillTest/Infill_generate_connect/1/400",
            "value": 2474991.109791798,
            "unit": "ns/iter",
            "extra": "iterations: 337\ncpu: 2474683.382789317 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generate_connect/0/400",
            "value": 12482495.190910082,
            "unit": "ns/iter",
            "extra": "iterations: 110\ncpu: 12481217.27272727 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generate_connect/1/800",
            "value": 50220895.20000463,
            "unit": "ns/iter",
            "extra": "iterations: 10\ncpu: 50215509.99999999 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generate_connect/0/800",
            "value": 252817186.2857432,
            "unit": "ns/iter",
            "extra": "iterations: 7\ncpu: 252782685.71428576 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generate_connect/1/1200",
            "value": 642090739.9999578,
            "unit": "ns/iter",
            "extra": "iterations: 2\ncpu: 642041449.9999998 ns\nthreads: 1"
          },
          {
            "name": "InfillTest/Infill_generate_connect/0/1200",
            "value": 1094698159.000018,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 1094601599.9999998 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/generateWalls/3",
            "value": 5770293.790001232,
            "unit": "ns/iter",
            "extra": "iterations: 100\ncpu: 5769792.000000003 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/generateWalls/15",
            "value": 17283641.591839857,
            "unit": "ns/iter",
            "extra": "iterations: 49\ncpu: 17282126.530612253 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/generateWalls/9999",
            "value": 39630512.300004736,
            "unit": "ns/iter",
            "extra": "iterations: 20\ncpu: 39627730.000000045 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getRegionOrder/3",
            "value": 139468.662070362,
            "unit": "ns/iter",
            "extra": "iterations: 5004\ncpu: 139455.33573141505 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getRegionOrder/15",
            "value": 2156828.88364807,
            "unit": "ns/iter",
            "extra": "iterations: 318\ncpu: 2156680.5031446526 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getRegionOrder/9999",
            "value": 6929331.326730763,
            "unit": "ns/iter",
            "extra": "iterations: 101\ncpu: 6928586.138613873 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getInsetOrder/3",
            "value": 1010.7683889281325,
            "unit": "ns/iter",
            "extra": "iterations: 683849\ncpu: 1010.7114289850539 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getInsetOrder/15",
            "value": 6078.7440611640895,
            "unit": "ns/iter",
            "extra": "iterations: 116016\ncpu: 6078.403840849551 ns\nthreads: 1"
          },
          {
            "name": "WallTestFixture/InsetOrderOptimizer_getInsetOrder/9999",
            "value": 19247.876942350456,
            "unit": "ns/iter",
            "extra": "iterations: 37519\ncpu: 19246.808283802813 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}